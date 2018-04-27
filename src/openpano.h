#pragma once
#include <string>
#include <vector>

namespace openpano {

bool stitch(
    const std::string& config_file,
    const std::string& outfile,
    const std::vector<std::string>& infiles);

}
