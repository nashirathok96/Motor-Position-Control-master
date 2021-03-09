using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using ZedGraph;

namespace Motor_Position_Control
{
    public partial class Form1 : Form
    {
        GraphPane myPane = new GraphPane();
        PointPairList refPoint = new PointPairList();
        PointPairList actPoint = new PointPairList();
        UInt64 n = 0;

        public Form1()
        {
            InitializeComponent();
        }

        void RefreshComPort()
        {
            if (serialPort.IsOpen) serialPort.Close();

            cbPort.Items.Clear();
            cbPort.Text = String.Empty;
            cbPort.Items.AddRange(SerialPort.GetPortNames());
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            Form1.CheckForIllegalCrossThreadCalls = false;
            RefreshComPort();
            serialPort.BaudRate = 115200;

            myPane.Title.Text = "PID Setpoint and Response";
            myPane.XAxis.Title.Text = "Time";
            myPane.XAxis.Title.Text = "Angle";
            myPane.Title.Text = "PID Setpoint and Response";
            myPane.XAxis.Title.Text = "Time";
            myPane.XAxis.Title.Text = "Angle";
            //myPane.YAxis.Scale.Min = 0;
            //myPane.YAxis.Scale.Max = 180;
            myPane.YAxis.Scale.MajorStepAuto = false;
            myPane.YAxis.Scale.MajorStep = 1;
            myPane.YAxis.Scale.MinorStepAuto = false;
            myPane.YAxis.Scale.MinorStep = 1;
            myPane.YAxis.MajorGrid.IsVisible = true;
            myPane.XAxis.Scale.MajorStepAuto = false;
            myPane.XAxis.Scale.MajorStep = 10;
            myPane.XAxis.MajorGrid.IsVisible = true;

            LineItem refCurve = myPane.AddCurve("Ref", refPoint, Color.Red, SymbolType.None);
            LineItem actCurve = myPane.AddCurve("Act", actPoint, Color.Blue, SymbolType.None);

            zgc.GraphPane = myPane;
            zgc.ClientSize = new Size(900, 450);
            zgc.AxisChange();
        }

        private void btnRefresh_Click(object sender, EventArgs e)
        {
            RefreshComPort();
        }

        private void btnConnect_Click(object sender, EventArgs e)
        {
            if (serialPort.IsOpen) serialPort.Close();
            if (cbPort.Text.Equals(String.Empty))
            {
                MessageBox.Show("Select port first before connect");
                return;
            }

            serialPort.PortName = cbPort.Text.Trim();
            serialPort.Open();
        }
        private void btnInput_Click(object sender, EventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                MessageBox.Show("Serial port not connected");
                return;
            }
            serialPort.WriteLine(inputRef.Value.ToString());
        }

        private void btnSetting_Click(object sender, EventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                MessageBox.Show("Serial port not connected");
                return;
            }
            serialPort.WriteLine("p" + float.Parse(inputKp.Text.Trim()).ToString().Replace(',','.'));
            Console.WriteLine("p" + float.Parse(inputKp.Text.Trim()).ToString().Replace(',', '.'));
        }

        private void btnSetting2_Click(object sender, EventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                MessageBox.Show("Serial port not connected");
                return;
            }
            serialPort.WriteLine("i" + float.Parse(inputKi.Text.Trim()).ToString().Replace(',', '.'));
            Console.WriteLine("i" + float.Parse(inputKi.Text.Trim()).ToString().Replace(',', '.'));
        }

        private void btnSetting3_Click(object sender, EventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                MessageBox.Show("Serial port not connected");
                return;
            }
            serialPort.WriteLine("d" + float.Parse(inputKd.Text.Trim()).ToString().Replace(',', '.'));
            Console.WriteLine("d" + float.Parse(inputKd.Text.Trim()).ToString().Replace(',', '.'));
        }

        private void serialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            String data = serialPort.ReadLine();
            Console.WriteLine(data);
            if (data.Contains("KP"))
            {
                label5.Text = data;
            }
            else if (data.Contains("KI"))
            {
                label6.Text = data;
            }
            else if (data.Contains("KD"))
            {
                label7.Text = data;
            }

            try
            {
                String[] dataIn = data.Split(',');
                if (dataIn.Length == 5)
                {
                    if (dataIn[0].Contains("h") && dataIn[4].Contains("l"))
                    {
                        refPoint.Add(n, double.Parse(dataIn[1]));
                        actPoint.Add(n, double.Parse(dataIn[2]));
                        n++;
                        zgc.AxisChange();
                        zgc.Invalidate();

                        if (refPoint.Count > 1000)
                        {
                            refPoint.RemoveAt(0);
                            actPoint.RemoveAt(0);
                        }
                    }
                }
            }
            catch { };
        }

    }
}
