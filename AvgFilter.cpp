#include <iostream>
#include <stdio.h>

double prevAvg = 0;
double k = 0;
double firstRun = 0;

double GetVolt()
{
    double w = (double)(rand()%100)/100;
    
    double z = 14.4+w;

    return z;
}

double AvgFilter(double x)
{
    if (firstRun==0)
    {
        k=1;
        prevAvg=0;
        firstRun=1;
    }
    double alpha = (k-1)/k;
    double avg = alpha*prevAvg+(1-alpha)*x;

    prevAvg = avg;
    k = k+1;

    return avg;
}

int main()
{
    for (int i=0; i<5;i++)
    {
        
        double xm = GetVolt();
        std::cout << "xm:" << xm << std::endl;
        double avg = AvgFilter(xm);

        std::cout << "avg :" << avg << std::endl;
    }
   
    return 0;
}