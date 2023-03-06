#include "exception/BaseException.h"

#include <cstring>

BaseException::BaseException(std::string error) : _error(std::move(error)) {
    _error += std::string(" : ") + strerror(errno);
}

const char *BaseException::what() const noexcept { return _error.c_str(); }
