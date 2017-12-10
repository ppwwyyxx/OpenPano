#pragma once
#include <string>
#include <vector>

namespace openpano {

void stitch(
    const std::string& config_file,
    const std::string& outfile,
    const std::vector<std::string>& infiles);

}
