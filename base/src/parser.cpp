/** \file parser.cpp
 * \brief Simple vector expression parser sourcefile
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

#include <vector>
#include <sstream>

#include <grl/configurable.h>
#include <grl/vector.h>
#include <grl/parser.h>

using namespace grl;

#define OPERATORS "+*,-"
#define WHITESPACE " \t"
#define OPENPAREN "[("
#define CLOSEPAREN "])"

static bool isempty(std::istringstream &iss)
{
  std::string rest;
  iss >> std::ws >> rest;
  return rest.empty();
}

std::string ASTNode::evaluate()
{
  std::string left, right;
  if (left_)
    left = left_->evaluate();
  if (right_)
    right = right_->evaluate();
    
  if (str_ == "[]")
    return "[ " + left + " ]";
    
  // Parse values
  std::istringstream issx, issy;
  std::vector<double> x_in, y_in, z_out;
  
  // TODO: check whether these are numbers/vectors
  issx.str(left);  issx >> x_in;
  issy.str(right); issy >> y_in;

  LargeVector x, y, z;
  if (isempty(issx))
    toVector(x_in, x);
  if (isempty(issy))
    toVector(y_in, y);
    
  if (!x.size() || !y.size())
    return left + str_ + right;

  if (str_ == "+")
  {
    if (x.size() == 1)             z = x[0] + y;
    else if (y.size() == 1)        z = x + y[0];
    else if (x.size() == y.size()) z = x + y;
    else
      ERROR("Cannot add " << x_in << " and " << y_in << ": vector size mismatch");
  }
  else if (str_ == "++")
  {
    z = extend(x, y);
  }
  else if (str_ == "-")
  {
    if (x.size() == 1)             z = x[0] - y;
    else if (y.size() == 1)        z = x - y[0];
    else if (x.size() == y.size()) z = x - y;
    else
      ERROR("Cannot subtract " << x_in << " and " << y_in << ": vector size mismatch");
  }
  else if (str_ == "--")
  {
    if (x.size() != 1 || y.size() != 1)
      ERROR("Cannot create list from " << x_in << " to " << y_in << ": requires scalar operands");
    else if (x[0] < y[0])
    {
      z.resize(y[0]-x[0]);
      for (size_t ii=0; ii != z.size(); ++ii)
        z[ii] = x[0] + ii;
    }
  }
  else if (str_ == "*")
  {
    if (x.size() == 1)             z = x[0] * y;
    else if (y.size() == 1)        z = x * y[0];
    else if (x.size() == y.size()) z = x * y;
    else
      ERROR("Cannot multiply " << x_in << " and " << y_in << ": vector size mismatch");
  }
  else if (str_ == "**")
  {
    if (y.size() == 1 && y[0] > 0) z = x.replicate(1, y[0]);
    else
      ERROR("Cannot replicate " << x_in << " " << y_in << " times: vector size mismatch");
  }
  else
    return left + str_ + right;
  
  fromVector(z, z_out);

  std::ostringstream oss;
  oss << std::setprecision(std::numeric_limits<double>::max_digits10);
  
  if (z.size() == 1) oss << z_out[0];
  else               oss << z_out;
  
  return oss.str();
}

ASTNodePtr grl::parseExpression(const std::string &_str)
{
  std::string str = _str;

  if (str.empty())
    return ASTNodePtr(new ASTNode());

  // Left-associative parsing    
  const char *cc = str.c_str();
  std::string op;
  ASTNodePtr node;
  while (true)
  {
    // Eat whitespace
    for (; *cc && strchr(WHITESPACE, *cc); ++cc);

    // Extract subexpression
    std::string expr;
    int paren = 0;
    for (; *cc && ((!strchr(OPERATORS, *cc) && !strchr(WHITESPACE, *cc)) || paren); ++cc)
    {
      expr.push_back(*cc);
      if (strchr(OPENPAREN, *cc)) paren++;
      else if (strchr(CLOSEPAREN, *cc) && !--paren)
      {
        ++cc;
        break;
      }
    }
    
    // Create node from subexpression 
    ASTNodePtr newnode;
    
    if (!expr.empty() && expr.front() == '[' && expr.back() == ']')
      newnode = ASTNodePtr(new ASTNode("[]", parseExpression(expr.substr(1, expr.size()-2))));
    else if (!expr.empty() && expr.front() == '(' && expr.back() == ')')
      newnode = ASTNodePtr(new ASTNode("", parseExpression(expr.substr(1, expr.size()-2))));
    else
      newnode = ASTNodePtr(new ASTNode(expr));

    // Add to AST      
    if (node)
      node = ASTNodePtr(new ASTNode(op, node, newnode));
    else
      node = newnode;

    // Eat whitespace
    for (; *cc && strchr(WHITESPACE, *cc); ++cc);

    if (*cc && strchr(OPERATORS, *cc))
    {
      // Expecting more
      op.clear();
      for(; strchr(OPERATORS, *cc); ++cc)
        op.push_back(*cc);
    }
    else
    {
      // Done
      if (*cc)
        return ASTNodePtr(new ASTNode(" ", node, parseExpression(str.substr(cc-str.c_str()))));
      else
        return node;
    }
  }
}
