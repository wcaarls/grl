/** \file parser.h
 * \brief Simple vector expression parser
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2019-03-29
 *
 * \copyright \verbatim
 * Copyright (c) 2019, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */

#include <string>
#include <ostream>
#include <memory>

namespace grl
{

class ASTNode;
typedef std::shared_ptr<ASTNode> ASTNodePtr;

class ASTNode
{
  protected:
    std::string str_;
    ASTNodePtr left_, right_;

  public:
    ASTNode(const std::string &str="", const ASTNodePtr &left=ASTNodePtr(), const ASTNodePtr &right=ASTNodePtr()) : str_(str), left_(left), right_(right) { }

    std::string evaluate();
    
    friend std::ostream& operator<<(std::ostream& os, const ASTNode& node)
    {
      os << "{op:'" << node.str_ << "'";
      if (node.left_)  os << ",l:" << *node.left_;
      if (node.right_) os << ",r:" << *node.right_;
      os << "}";
      return os;
    }
};

ASTNodePtr parseExpression(const std::string &str);

inline std::string evaluateExpression(const std::string &str)
{
  // Expressions starting with @ are not evaluated.
  if (!str.empty() && str[0] == '@')
    return str;

  ASTNodePtr ast = parseExpression(str);
  return ast->evaluate();
}

}
