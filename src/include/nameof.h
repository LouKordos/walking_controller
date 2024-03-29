/****************************************************************************
** nameof() Macros
** version 1.0.0
** https://github.com/bravikov/nameof
**
** MIT License
**
** Copyright (c) 2018 Dmitry Bravikov <dmitry@bravikov.pro>
**
** Permission is hereby granted, free of charge, to any person obtaining a
** copy of this software and associated documentation files (the "Software"),
** to deal in the Software without restriction, including without limitation
** the rights to use, copy, modify, merge, publish, distribute, sublicense,
** and/or sell copies of the Software, and to permit persons to whom the
** Software is furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included
** in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
** OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
** DEALINGS IN THE SOFTWARE.
**
****************************************************************************/

#pragma once
#ifndef NAMEOF_H_20180129_123322
#define NAMEOF_H_20180129_123322


#define nameof(x) bravikov::_nameof<0>(#x, sizeof(x))


#include <string>
#include <regex>
#include <stdexcept>

namespace bravikov {
    template<int a>
    std::string _nameof(const std::string &x, std::size_t)
    {
        std::regex regex("^&?([_a-zA-Z]\\w*(->|\\.|::))*([_a-zA-Z]\\w*)$");
        std::smatch match;
        if (std::regex_match(x, match, regex)) {
            if (match.size() == 4) {
                return match[3];
            }
        }
        throw std::logic_error(
            "A bad expression x in nameof(x). The expression is \"" + x + "\"."
        );
    }
}

#endif /* #ifndef NAMEOF_H_20180129_123322 */