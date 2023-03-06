#ifndef UDP_TRANSLATOR_BASEEXCEPTION_H
#define UDP_TRANSLATOR_BASEEXCEPTION_H

#include <exception>
#include <string>

class BaseException : public std::exception {
 public:
    explicit BaseException(std::string error);
    [[nodiscard]] const char* what() const noexcept override;

 private:
    std::string _error;
};

#endif  // UDP_TRANSLATOR_BASEEXCEPTION_H
