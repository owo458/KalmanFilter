

#include <iostream>
#include <stdio.h>
#include <vector>

using namespace std;

template<typename AlldataType, std::size_t N>
AlldataType sumArr(const AlldataType (&data)[N]) 
{
    AlldataType total = AlldataType();

    for (std::size_t i = 0; i < N; ++i) 
    {
        total += data[i];
    }

    return total;
}

template<typename AlldataType>
AlldataType sumVector(const std::vector<AlldataType>& data) 
{
    AlldataType total = AlldataType();

    for (const auto& value : data) 
    {
        total += value;
    }

    return total;
}

int n = 0;
double prevAvg = 0;
double xbuf[] = {0,};
double firstRun = 0;

double MovAvgFilter(double x)
{
    if (firstRun==0)
    {
        n=10;
        for(int i =0; i<n; i++)
        {
            xbuf[i]=1;
        }
        int k =1;
        prevAvg = x;
        firstRun=1;
    }
    
    for (int m=1; m < n-1; m++)
    {
        xbuf[m] = xbuf[m+1];
    }
    xbuf[n+1]=x;

    double avg = prevAvg + (x-xbuf[1])/n;

    prevAvg = avg;

    return avg;
}

double GetVolt()
{
    double w = (double)(rand()%100)/100;
    
    double z = 14.4+w;

    return z;
}

int main()
{
    for (int i=0; i<5;i++)
    {
        
        double xm = GetVolt();
        std::cout << "xm:" << xm << std::endl;
        double MovAvg = MovAvgFilter(xm);

        std::cout << "MovAvg :" << MovAvg << std::endl;
    }
}