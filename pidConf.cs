using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NGCP.IO
{

    public class pidConf
    {
        public int index;
        public double Pi;
        public double Ii;
        public double Di;
        public double desval;
        public pidConf(int indedx, double p, double i, double d,double des)
        {
            index = indedx; Pi = p; Ii = i; Di = d; desval = des;
        }

        public pidConf(int indedx, double p, double i, double d) : this(indedx,p,i,d,0){ }

        public pidConf():this(-1,.8,0,.2,0) {}
    }
}
