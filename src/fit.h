#ifndef _FIT_H_
#define _FIT_H_

#include <string>
#include <vector>

class FloatPoint
{
  public:
    float x, y;
    FloatPoint(float x, float y)
    {
        this->x = x;
        this->y = y;
    }
};

class Equation
{
  private:
    float *c;
    void gaussEliminationLS(int m, int n, float **a, float *x);

  public:
    int deg;
    Equation();
    ~Equation();
    float evaluate(float x);
    void fit(int n, std::vector<FloatPoint> &data);
    std::string toString();
};

#endif
