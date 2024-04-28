using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading;

public class UsbCom : IDisposable
{
	public enum RunMode { Configuration, FrameTransfer, Quit };

    SerialPort serialPort;
	RunMode running = RunMode.Configuration;

    int adc_resolution = 0;
    int adc_buffer_length = 0;
    int adc_samples_per_second = 0;

	Stopwatch? stopwatch = null;
    int bufCounter;  // number of buffer received
	byte[]? buffer;
	int bufLen;
	int bufPos;

    FileStream? fs;

    static public void Main()
    {
        UsbCom usbCom = new UsbCom();
        usbCom.Run();
    }

    UsbCom()
    {
        // Create a new SerialPort object with default settings.
        serialPort = new SerialPort();
        serialPort.PortName = SetPortName(serialPort.PortName); // "/dev/ttyACM0";
        serialPort.BaudRate = 230400;  // 9600,38400, 57600, 115200, 230400, 250000, 500000, 1000000, 2000000
        serialPort.Parity = SetPortParity(serialPort.Parity);
        serialPort.DataBits = 8;
        serialPort.StopBits = SetPortStopBits(serialPort.StopBits);
        serialPort.Handshake = SetPortHandshake(serialPort.Handshake);

        // serialPort.ReadTimeout = 500;  // ms
        // serialPort.WriteTimeout = 500;  // ms
		serialPort.ReadBufferSize = 4096*16;  // default/min 4096
		// serialPort.WriteBufferSize = 2048;  // default/min 2048
		Console.WriteLine($"Timeout {serialPort.ReadTimeout} / {serialPort.WriteTimeout} ms");
		Console.WriteLine($"BufferSize {serialPort.ReadBufferSize} / {serialPort.WriteBufferSize} bytes");

		serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
		serialPort.ErrorReceived += new SerialErrorReceivedEventHandler(ErrorReceivedHandler);
        serialPort.Open();

        bufCounter = 0;
        bufPos = 0;
        bufLen = 0;
        buffer = null;

        fs = new FileStream("data.out", FileMode.Create, FileAccess.Write);
    }

    public void Dispose()
    {
        if (fs is not null)
            fs.Dispose();
    }

    void Run()
    {
        // Receive messages from device (either through thread or events)
        // Thread readThread = new Thread(Read);
		// readThread.IsBackground = true;
        // readThread.Start();

		// Send commands to the device
		Console.WriteLine("Type INFO to get current status, QUIT to exit");

        while (running != RunMode.Quit)
        {
			Console.Write("$ ");
			string cmd = Console.ReadLine() ?? "";
            if (StringComparer.OrdinalIgnoreCase.Equals("info", cmd))
            {
                ShowHostConfiguration();

                // Show device configuration
				if (running == RunMode.Configuration)
                {
                    // flush late coming buffers
                    serialPort.DiscardInBuffer();

					// request (and display) information
					serialPort.WriteLine(cmd);
                }
            }
            else if (StringComparer.OrdinalIgnoreCase.Equals("start", cmd))
            {
                if (buffer == null)
                {
                    Console.WriteLine("! buffer is null, run 'info' first.");
                    continue;
                }

                running = RunMode.FrameTransfer;
                stopwatch = Stopwatch.StartNew();
                bufCounter = 0;
                serialPort.WriteLine(cmd);
            }
            else if (StringComparer.OrdinalIgnoreCase.Equals("stop", cmd))
            {
                ShowHostConfiguration();

                running = RunMode.Configuration;
                serialPort.WriteLine(cmd);
            }
            else if (StringComparer.OrdinalIgnoreCase.Equals("quit", cmd))
            {
                running = RunMode.Quit;
                fs.Close();
                serialPort.Close();
            }
            else
            {
                serialPort.WriteLine(cmd);
            }
        }


        // readThread.Join();
        serialPort.Close();
    }

	public void ErrorReceivedHandler(
		object sender,
		SerialErrorReceivedEventArgs e)
	{
		// EventType SerialError
		// Frame    	8   	The hardware detected a framing error.
		// Overrun  	2   	A character-buffer overrun has occurred. The next character is lost.
		// RXOver   	1   	An input buffer overflow has occurred. There is either no room in the input buffer, or a character was received after the end-of-file (EOF) character.
		// RXParity 	4   	The hardware detected a parity error.
		// TXFull   	256 	The application tried to transmit a character, but the output buffer was full.
		Console.WriteLine($"ErrorReceivedHandler {e.EventType}");
    }

	private void DataReceivedHandler(
		object sender,
        SerialDataReceivedEventArgs e)
    {
		// EventType SerialData
		// Chars    	1   	A character was received and placed in the input buffer.
		// Eof      	2   	The end of file character was received and placed in the input buffer.
		// Console.WriteLine($"DataReceivedHandler {e.EventType}");

        SerialPort sp = (SerialPort)sender;

		if (running != RunMode.FrameTransfer || buffer == null)
        {
			// default communication mode is text
            while (sp.BytesToRead != 0)
            {
                // Console.WriteLine($"BytesToRead: {sp.BytesToRead}");
    			string status = sp.ReadLine();
	    		Console.WriteLine($"Status Received: {status}");
                ProcessStatusMessage(status);
            }
            UpdateConfiguration();
		}
		else
		{
            // Console.WriteLine($"Bytes to read: #{sp.BytesToRead}");
			int bufIncr = sp.Read(buffer, bufPos, bufLen - bufPos);
			bufPos += bufIncr;
			if (bufPos == bufLen) {
				bufPos = 0;
                BufferReceived();

			}
            // Console.WriteLine($"Bytes fetched: #{bufIncr}");
		}
    }

    private void ShowHostConfiguration()
    {
        Console.WriteLine($"Host RunMode: {running}");
        if (running == RunMode.FrameTransfer && stopwatch is not null)
        {
            var time_ms = stopwatch.ElapsedMilliseconds;
            var kSps = bufCounter * adc_buffer_length * (adc_resolution > 8 ? 0.5 : 1.0) * ( 1000.0 / time_ms );
            Console.WriteLine($"Host buffers received: #{bufCounter}");
            Console.WriteLine($"Host timme elapsed: {time_ms}ms");
            Console.WriteLine($"Host sample per seconds: {kSps}kSps");
        }
    }

    private void ProcessStatusMessage(string status)
    {
        string[] kv = status.Split('\t', 2);
        if (kv.Length != 2)
            return;


        string key = kv[0].Trim(), val = kv[1].Trim();
        if (StringComparer.OrdinalIgnoreCase.Equals(key, "USB_ADC_TRANSMIT"))
        {
            int deviceTransmit = int.Parse(val);
            if (deviceTransmit != 0)
                throw new Exception($"USB ADC Device is transmitting");
        }
        else if (StringComparer.OrdinalIgnoreCase.Equals(key, "USB_ERROR"))
        {
            //if (!string.IsNullOrEmpty(val) && val != "OK")
            //    throw new Exception($"USB ADC Device is in error");

        }
        if (StringComparer.OrdinalIgnoreCase.Equals(key, "ADC_RESOLUTION"))
            adc_resolution = int.Parse(val);
        else if (StringComparer.OrdinalIgnoreCase.Equals(key, "USB_ADC_BUF_LEN"))
            adc_buffer_length = int.Parse(val);
        else if (StringComparer.OrdinalIgnoreCase.Equals(key, "ADC_SAMPLE_PER_SEC"))
            adc_samples_per_second = int.Parse(val);
        else if (StringComparer.OrdinalIgnoreCase.Equals(key, "USB_ADC_ERR"))
            throw new Exception($"USB ADC Error: {val}");
        else
            Console.WriteLine($"Unhandled status key '{key}'.");
    }

    private void UpdateConfiguration()
    {
        // Buffer size Calculation should match device (header added to data)

        bufLen = adc_buffer_length;
        buffer = new byte[bufLen];

        bufCounter = 0;
        bufPos = 0;

        Console.WriteLine($"ADC Buffer size set to {bufLen} bytes");
        Console.WriteLine();

        Console.Write("$ ");
    }

	private void BufferReceived()
	{
		// To do : check the bufCounter at the begining of the buffer matches

		bufCounter++;

        if (buffer is not null)
        {
            int deviceCounter = buffer[0] << 3 | buffer[1] << 2 | buffer[2] << 1 | buffer[3];
            Console.WriteLine($"BufferReceived {buffer[0]} {buffer[1]} {buffer[2]} {buffer[3]} / {deviceCounter}  =>  {buffer[8]} {buffer[bufLen>>1]} ");

            if (fs is not null)
                fs.Write(buffer, 0, buffer.Length);
        }
	}


    public void Read()
    {
		/*
        while (running)
        {
            try
            {
                string message = serialPort.ReadLine();
                Console.WriteLine(message);
            }
            catch (TimeoutException) { }
        }
		*/
    }

    // Display Port values and prompt user to enter a port.
    public string SetPortName(string defaultPortName)
    {
        Console.WriteLine("Available Ports:");
        foreach (string s in SerialPort.GetPortNames())
        {
            Console.WriteLine("   {0}", s);
        }

        if (!SerialPort.GetPortNames().Contains(defaultPortName))
			defaultPortName = SerialPort.GetPortNames().First();

		Console.WriteLine($"Used Port option: {defaultPortName}");
		return defaultPortName;
    }

    // Display PortParity values and prompt user to enter a value.
    public Parity SetPortParity(Parity defaultPortParity)
    {
        Console.WriteLine("Available Parity options:");
        foreach (string s in Enum.GetNames(typeof(Parity)))
        {
            Console.WriteLine("   {0}", s);
        }

        Console.WriteLine($"Used Parity option: {defaultPortParity}");
        return defaultPortParity;
    }


    // Display StopBits values and prompt user to enter a value.
    public StopBits SetPortStopBits(StopBits defaultPortStopBits)
    {
        Console.WriteLine("Available StopBits options:");
        foreach (string s in Enum.GetNames(typeof(StopBits)))
        {
            Console.WriteLine("   {0}", s);
        }

        Console.WriteLine($"Used StopBits option: {defaultPortStopBits}");
        return defaultPortStopBits;
    }

    public Handshake SetPortHandshake(Handshake defaultPortHandshake)
    {
        Console.WriteLine("Available Handshake options:");
        foreach (string s in Enum.GetNames(typeof(Handshake)))
        {
            Console.WriteLine("   {0}", s);
        }

        Console.WriteLine($"Used Handshake option: {defaultPortHandshake}");
        return defaultPortHandshake;
    }
}
