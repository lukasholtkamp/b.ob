#include <bits/stdc++.h> 
#include <stdint.h> //<-- Used to define the uint

class rolling_median{
    public:
      std::deque<u_int32_t> arr;
      int window_size;

      rolling_median(int ws){
        window_size = ws;
      }

      void insert(u_int32_t x){
        if (arr.size()>=(unsigned)window_size){
          arr.pop_front();
          arr.push_back(x);
        }
        else{
          arr.push_back(x);
        }
      }

      double getMedian(){

        std::deque<u_int32_t>tmp = arr;
        
        // Sort the deque
        sort(tmp.begin(), tmp.end());

        // finding size of deque
        int n = tmp.size();

        // Check if the number of elements is odd
        double median;
        if (n % 2 != 0)
            median = (double)tmp[n / 2];
        else
        // If the number of elements is even, return the
        // average of the two middle elements
        median = (double)(tmp[(n - 1) / 2] + tmp[n / 2]) / 2.0;


        return median;
      }

  };