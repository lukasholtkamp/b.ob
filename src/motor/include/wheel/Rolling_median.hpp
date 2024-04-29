#include <bits/stdc++.h> 
#include <stdint.h> //<-- Used to define the uint

class rolling_median{
    public:
      std::list <u_int32_t> arr;
      int window_size;

      rolling_median(int ws){
        window_size = ws;
      }

      void insert(u_int32_t x){
        if (arr.size()>=window_size){
          arr.pop_front();
          arr.push_back(x);
        }
        else{
          arr.push_back(x);
        }
      }

      double getMedian(){

        std::list <u_int32_t> tmp = arr;

        tmp.sort();

        tmp

        return (a + b) * 0.5;
      }

  };