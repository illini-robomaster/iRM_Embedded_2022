%--------------------------------------------------------------------------%
%                                                                          %
%  Copyright (C) 2022 RoboMaster.                                          %
%  Illini RoboMaster @ University of Illinois at Urbana-Champaign          %
%                                                                          %
%  This program is free software: you can redistribute it and/or modify    %
%  it under the terms of the GNU General Public License as published by    %
%  the Free Software Foundation, either version 3 of the License, or       %
%  (at your option) any later version.                                     %
%                                                                          %
%  This program is distributed in the hope that it will be useful,         %
%  but WITHOUT ANY WARRANTY; without even the implied warranty of          %
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           %
%  GNU General Public License for more details.                            %
%                                                                          %
%  You should have received a copy of the GNU General Public License       %
%  along with this program. If not, see <http://www.gnu.org/licenses/>.    %
%                                                                          %
%--------------------------------------------------------------------------%

magData = squeeze(out.mag)';
[A,b,exp] = magcal(magData, "auto");

figure;
plot3(magData(:, 1), magData(:, 2), magData(:, 3), Marker='X', LineStyle='none');
hold on;

C = (magData - b)*A;
plot3(C(:, 1), C(:, 2), C(:, 3), LineStyle='none', Marker='o');
axis equal;
hold off;

figure;
hold on;
plot(vecnorm(magData'));
plot(vecnorm(C'));
hold off;
