/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_NODES_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_NODES_H_

#include "nodes.h"

namespace towr {

/**
 * @brief Node variables used to construct the base motion spline.
 *
 * Every node is optimized over, in contrast to PhaseNodes, where multiple
 * nodes in the spline are represented by the same optimization variables.
 */
class BaseNodes : public Nodes {
public:
  /**
   * @param n_nodes  The number of nodes to construct the spline.
   * @param variable_id  The name of this variables set in the optimization.
   */
  BaseNodes (int n_nodes, std::string variable_id);
  virtual ~BaseNodes () = default;

  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const override;
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_NODES_H_ */