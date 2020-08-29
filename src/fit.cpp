#include "fit.h"
#include <cmath>
#include <sstream>

#include <ros/ros.h>

using namespace std;

Equation::Equation()
{
    deg = 0;
    c = NULL;
}

Equation::~Equation()
{
    deg = 0;
    if (c)
        delete[] c;
}

float Equation::evaluate(float x)
{
    float y = 0;
    for (int i = 0; i <= deg; i++)
        y += c[i] * pow(x, i);
    return y;
}

string Equation::toString()
{
    stringstream ss;
    for (int i = deg; i >= 0; i--)
    {
        if (i == deg)
        {
            ss << c[i];
        }
        else
        {
            if (c[i] < 0)
                ss << " - " << abs(c[i]);
            else
                ss << " + " << abs(c[i]);
        }
        if (i > 0)
            ss << "x^" << i;
    }
    return ss.str();
}

void Equation::fit(int n, std::vector<FloatPoint> &data)
{
    int N = data.size();
    float *x = new float[N];
    float *y = new float[N];

    int i, j;
    for (i = 0; i < N; i++)
    {
        x[i] = data[i].x;
        y[i] = data[i].y;
    }

    // an array of size 2*n+1 for storing N, Sig xi, Sig xi^2, ...., etc. which are the independent components of the
    // normal matrix
    float *X = new float[2 * n + 1];
    for (i = 0; i <= 2 * n; i++)
    {
        X[i] = 0;
        for (j = 0; j < N; j++)
        {
            X[i] = X[i] + pow(x[j], i);
        }
    }
    // the normal augmented matrix
    float **B = new float *[n + 1];
    for (i = 0; i < n + 1; i++)
    {
        B[i] = new float[n + 2];
    }
    // rhs
    float *Y = new float[n + 1];
    for (i = 0; i <= n; i++)
    {
        Y[i] = 0;
        for (j = 0; j < N; j++)
        {
            Y[i] = Y[i] + pow(x[j], i) * y[j];
        }
    }
    for (i = 0; i <= n; i++)
    {
        for (j = 0; j <= n; j++)
        {
            B[i][j] = X[i + j];
        }
    }
    for (i = 0; i <= n; i++)
    {
        B[i][n + 1] = Y[i];
    }
    if (c)
        delete[] c;
    c = new float[n + 1];

    gaussEliminationLS(n + 1, n + 2, B, c);
    deg = n;

    delete[] Y;
    for (i = 0; i < n + 1; i++)
        delete[] B[i];
    delete[] B;
    delete[] X;
    delete[] x;
    delete[] y;
}

void Equation::gaussEliminationLS(int m, int n, float **a, float *x)
{
    int i, j, k;
    for (i = 0; i < m - 1; i++)
    {
        // Partial Pivoting
        for (k = i + 1; k < m; k++)
        {
            // If diagonal element(absolute vallue) is smaller than any of the terms below it
            if (fabs(a[i][i]) < fabs(a[k][i]))
            {
                // Swap the rows
                for (j = 0; j < n; j++)
                {
                    float temp;
                    temp = a[i][j];
                    a[i][j] = a[k][j];
                    a[k][j] = temp;
                }
            }
        }
        // Begin Gauss Elimination
        for (k = i + 1; k < m; k++)
        {
            float term = a[k][i] / a[i][i];
            for (j = 0; j < n; j++)
            {
                a[k][j] = a[k][j] - term * a[i][j];
            }
        }
    }
    // Begin Back-substitution
    for (i = m - 1; i >= 0; i--)
    {
        x[i] = a[i][n - 1];
        for (j = i + 1; j < n - 1; j++)
        {
            x[i] = x[i] - a[i][j] * x[j];
        }
        x[i] = x[i] / a[i][i];
    }
}