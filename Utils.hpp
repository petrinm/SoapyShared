#pragma once

#include <string>
#include <vector>

namespace SHMRegistry
{

    /* List all SHMs */
    std::vector<std::string> list();

    /* */
    void add(const std::string &name);

    /* */
    void remove(const std::string &name);

};