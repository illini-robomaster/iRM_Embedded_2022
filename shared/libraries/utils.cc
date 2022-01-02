/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2020 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "utils.h"

BoolEdgeDetecter::BoolEdgeDetecter(bool initial) {
  prev_ = initial;
}

void BoolEdgeDetecter::input(bool signal) {
  posEdge_ = false;
  negEdge_ = false;
  if (!prev_ && signal) 
    posEdge_ = true;
  else if (prev_ && !signal) 
    negEdge_ = true;
  prev_ = signal;
}

bool BoolEdgeDetecter::edge() { return posEdge_ || negEdge_; }

bool BoolEdgeDetecter::posEdge() { return posEdge_; }

bool BoolEdgeDetecter::negEdge() { return negEdge_; }

FloatEdgeDetecter::FloatEdgeDetecter(float initial, float threshold) {
  prev_ = initial;
  threshold_ = threshold;
}

void FloatEdgeDetecter::input(float signal) {
  posEdge_ = false;
  negEdge_ = false;
  float diff = signal - prev_;
  if (diff > threshold_)
    posEdge_ = true;
  else if (diff < -threshold_)
    negEdge_ = true;
  prev_ = signal;
}

bool FloatEdgeDetecter::edge() { return posEdge_ || negEdge_; }

bool FloatEdgeDetecter::posEdge() { return posEdge_; }

bool FloatEdgeDetecter::negEdge() { return negEdge_; }