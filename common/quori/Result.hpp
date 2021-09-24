#ifndef _QUORI_RESULT_HPP_
#define _QUORI_RESULT_HPP_

namespace quori
{
  template<typename T>
  class Result
  {
  public:
    Result(const T &ok)
      : type_(Type::Ok)
      , ok_(ok)
    {
    }
    
    Result(T &&ok)
      : type_(Type::Ok)
      , ok_(ok)
    {
    }

    Result()
      : type_(Type::Err)
      , ok_(T())
    {
    }
    
    bool isOk() const
    {
      return type_ == Type::Ok;
    }

    bool isErr() const
    {
      return type_ == Type::Err;
    }

    T &ok()
    {
      return ok_;
    }

    const T &ok() const
    {
      return ok_;
    }

  private:
    enum class Type { Ok, Err };

    Type type_;
    T ok_;
  };
}

#endif