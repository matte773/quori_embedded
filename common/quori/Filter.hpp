#ifndef _QUORI_FILTER_HPP_
#define _QUORI_FILTER_HPP_

#include <algorithm>
#include <array>
#include <vector>

namespace quori
{
  // A low-pass filter
  template<typename T>
  class Filter
  {
  public:
    Filter(const T factor)
      : factor_(factor)
      , current_(0.0)
      , inited_(false)
    {
    }

    T update(const T value)
    {
      if (!inited_)
      {
        current_ = value;
        inited_ = true;
        return current_;
      }

      current_ = value * factor_ + (1.0 - factor_) * current_;
      return current_;
    }

  private:
    T factor_;
    T current_;
    bool inited_;
  };

  // Median filter
  template<typename T, size_t N>
  class MedianFilter
  {
  public:
    MedianFilter()
      : i_(0)
      , inited_(false)
    {
      for (size_t i = 0; i < N; ++i)
      {
        values_[i] = T();
      }
    }

    T update(const T value)
    {
      if (!inited_)
      {
        for (size_t i = 0; i < N; ++i)
        {
          values_[i] = value;
        }
        inited_ = true;
        return value;
      }

      values_[i_] = value;
      i_ = (i_ + 1) % N;
      

      // Find the median without mutating the array
      std::array<T, N> sorted(values_);
      std::sort(sorted.begin(), sorted.end());
      return sorted[N / 2];
    }
  private:
    std::array<T, N> values_;
    size_t i_;
    bool inited_;
  };

}

#endif