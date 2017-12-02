#ifndef EXCEPTION_H
#define EXCEPTION_H
#include <exception>
#include <string>

class nt_exception : public std::exception
{
public:
    nt_exception(std::string&& msg) : m_msg(std::forward<std::string&&>(msg))
    {
    }

    virtual const char* what() const noexcept
    {
        return m_msg.c_str();
    }

private:
    const std::string m_msg;
};

#endif // EXCEPTION_H
