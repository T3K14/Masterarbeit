#include <iostream>
#include <array>

const int N = 6;
const int M = 3;



bool twiddle(int & x, int & y, int & z, std::array<int, N+2> & p) {

    int i, j, k;
    j = 1;
    while(p[j] <= 0)
        j++;
    if(p[j-1] == 0) {
        for(i = j-1; i != 1; i--)
            p[i] = -1;
        p[j] = 0;
        x = z = 0;
        p[1] = 1;
        y = j-1;
    }
    else {
        if(j > 1)
            p[j-1] = 0;
        do
            j++;
        while(p[j] > 0);
        k = j-1;
        i = j;
        while(p[i] == 0)
            p[i++] = -1;
        if(p[i] == -1) {
            p[i] = p[k];
            z = p[k]-1;
            x = i-1;
            y = k-1;
            p[k] = -1;
        }
        else {
            if(i == p[0])
	            return(1);
            else {
	            p[j] = p[i];
	            z = p[i]-1;
	            p[i] = 0;
	            x = j-1;
	            y = i-1;
	        }
        }
    }
    return(0);
}

void inittwiddle(int m, int n, std::array<int, N+2> & p) {
  int i;
  p[0] = n+1;
  for(i = 1; i != n-m+1; i++)
    p[i] = 0;
  while(i != n+1)
    {
    p[i] = i+m-n;
    i++;
    }
  p[n+1] = -2;
  if(m == 0)
    p[1] = 1;
  }


int main() {
    int x,y,z;
    std::array<int, N + 2> pExt;

    inittwiddle(M, N, pExt);

    twiddle(x,y,z, pExt);
    twiddle(x,y,z, pExt);

    twiddle(x,y,z, pExt);
    twiddle(x,y,z, pExt);
    twiddle(x,y,z, pExt);
    twiddle(x,y,z, pExt);
    twiddle(x,y,z, pExt);

    twiddle(x,y,z, pExt);
    twiddle(x,y,z, pExt);
    twiddle(x,y,z, pExt);
    twiddle(x,y,z, pExt);


}