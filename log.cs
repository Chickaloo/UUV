using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NGCP.IO
{
    public class log
    {
        public log()
        {

        }
        public void LOG(string d, int type)
        {
            var dt = DateTime.Now;
            using (System.IO.StreamWriter writetext = new System.IO.StreamWriter("RunLog-"+ dt.Date.ToShortDateString().Replace("/", "_") + ".txt",true))
            {
                var n = "[" + dt.ToString() + "] " + d;
                writetext.WriteLine(n);
                // Console.WriteLine(n);
            }
        }
    }
}
