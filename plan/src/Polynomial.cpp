#include <planning/Polynomial.h>


double Polynomial::CalculateValue(double param)
{
    double value=0;
    for(int i=0;i<coefficient.size();++i)
    {
        value += coefficient[i]*pow(param,i);
    }
    return value;

}
double Polynomial::CalculateFirstDerivative(double param)
{
    double value=0;
    for(int i=1;i<coefficient.size();++i)
    {
        value += i*coefficient[i]*pow(param,i-1);
    }
    return value;
}
double Polynomial::CalculatesecondDerivative(double param)
{
    double value=0;
    for(int i=2;i<coefficient.size();++i)
    {
        value += (i-1)*i*coefficient[i]*pow(param,i-2);
    }
    return value;
}
double Polynomial::CalculatethirdDerivative(double param)
{
    double value=0;
    for(int i=3;i<coefficient.size();++i)
    {
        value += (i-2)*(i-1)*i*coefficient[i]*pow(param,i-3);
    }
    return value;   
}