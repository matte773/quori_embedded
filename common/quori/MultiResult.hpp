#ifndef _QUORI_MULTI_RESULT_HPP_
#define _QUORI_MULTI_RESULT_HPP_

namespace quori
{
  template<typename S, typename T>
  class MultiResult
  {
  public:
    MultiResult(const S &state, const T &value)
      : state_(state)
      , value_(value)
    {
    }

    const S &getState() const
    {
      return state_;
    }

    T &value()
    {
      return value_;
    }

    const T &value() const
    {
      return value_;
    }

  private:
    S state_;
    T value_;
  };
}

#endif