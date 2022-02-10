#pragma once

#include <string>
#include <array>

enum JacobianIKMode {
	JT = 0, JPI = 1, DLS = 2, SVD = 3, SVD_DLS = 4, SDLS = 5, size = 6
};

const std::array<std::string, JacobianIKMode::size> JIK_MODE_NAMES{
	"JT", "JPI", "DLS", "SVD", "SVD_DLS", "SDLS"
};
