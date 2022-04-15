/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, 2022 Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *          chendingwei
 *********************************************************************/
#include "costmap.h"

using namespace std;

namespace costmap_2d
{
Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                                double origin_x, double origin_y, unsigned char default_value):
    size_x_(cells_size_x),
    size_y_(cells_size_y),
    resolution_(resolution),
    origin_x_(origin_x),
    origin_y_(origin_y),
    costmap_(NULL),
    default_value_(default_value)
{
    access_ = new mutex_t();

    // create the costmap
    initMaps(size_x_, size_y_);
    resetMaps();
}

void Costmap2D::deleteMaps()
{
    // clean up data
    boost::unique_lock<mutex_t> lock(*access_);
    delete[] costmap_;
    costmap_ = NULL;
}

void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
    boost::unique_lock<mutex_t> lock(*access_);
    delete[] costmap_;
    costmap_ = new unsigned char[size_x * size_y];
}

void Costmap2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                                double origin_x, double origin_y)
{
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    initMaps(size_x, size_y);

    // reset our maps to have no information
    resetMaps();
}

void Costmap2D::resetMaps()
{
    boost::unique_lock<mutex_t> lock(*access_);
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
    boost::unique_lock<mutex_t> lock(*(access_));
    unsigned int len = xn - x0;
    for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
        memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
}

Costmap2D& Costmap2D::operator=(const Costmap2D& map)
{
    // check for self assignement
    if (this == &map)
        return *this;

    // clean up old data
    deleteMaps();

    size_x_ = map.size_x_;
    size_y_ = map.size_y_;
    resolution_ = map.resolution_;
    origin_x_ = map.origin_x_;
    origin_y_ = map.origin_y_;

    // initialize our various maps
    initMaps(size_x_, size_y_);

    // copy the cost map
    memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

    return *this;
}

Costmap2D::Costmap2D(const Costmap2D& map) :
    costmap_(NULL)
{
    access_ = new mutex_t();
    *this = map;
}

// just initialize everything to NULL by default
Costmap2D::Costmap2D() :
    size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
    access_ = new mutex_t();
}

Costmap2D::~Costmap2D()
{
    deleteMaps();
    delete access_;
}

unsigned int Costmap2D::cellDistance(double world_dist)
{
    double cells_dist = max(0.0, ceil(world_dist / resolution_));
    return (unsigned int)cells_dist;
}

unsigned char* Costmap2D::getCharMap() const
{
    return costmap_;
}

unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
    return costmap_[getIndex(mx, my)];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
    costmap_[getIndex(mx, my)] = cost;
}

void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
    if (wx < origin_x_ || wy < origin_y_)
        return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
        return true;

    return false;
}

void Costmap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const
{
    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
}

void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const
{
    // Here we avoid doing any math to wx,wy before comparing them to
    // the bounds, so their values can go out to the max and min values
    // of double floating point.
    if (wx < origin_x_)
    {
        mx = 0;
    }
    else if (wx >= resolution_ * size_x_ + origin_x_)
    {
        mx = size_x_ - 1;
    }
    else
    {
        mx = (int)((wx - origin_x_) / resolution_);
    }

    if (wy < origin_y_)
    {
        my = 0;
    }
    else if (wy >= resolution_ * size_y_ + origin_y_)
    {
        my = size_y_ - 1;
    }
    else
    {
        my = (int)((wy - origin_y_) / resolution_);
    }
}

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
    // project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);

    // Nothing to update
    if (cell_ox == 0 && cell_oy == 0)
        return;

    // compute the associated world coordinates for the origin cell
    // because we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    // To save casting from unsigned int to int a bunch of times
    int size_x = size_x_;
    int size_y = size_y_;

    // we need to compute the overlap of the new and existing windows
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = min(max(cell_ox, 0), size_x);
    lower_left_y = min(max(cell_oy, 0), size_y);
    upper_right_x = min(max(cell_ox + size_x, 0), size_x);
    upper_right_y = min(max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    // we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];

    // copy the local window in the costmap to the local map
    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    // now we'll set the costmap to be completely unknown if we track unknown space
    resetMaps();

    // update the origin with the appropriate world coordinates
    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;

    // compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    // now we want to copy the overlapping information back into the map, but in its new location
    copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

    // make sure to clean up
    delete[] local_map;
}

unsigned int Costmap2D::getSizeInCellsX() const
{
    return size_x_;
}

unsigned int Costmap2D::getSizeInCellsY() const
{
    return size_y_;
}

double Costmap2D::getSizeInMetersX() const
{
    return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getOriginX() const
{
    return origin_x_;
}

double Costmap2D::getOriginY() const
{
    return origin_y_;
}

double Costmap2D::getResolution() const
{
    return resolution_;
}

bool Costmap2D::saveMap(std::string file_name)
{
    FILE *fp = fopen(file_name.c_str(), "w");

    if (!fp)
    {
        return false;
    }

    fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
    for (unsigned int iy = 0; iy < size_y_; iy++)
    {
        for (unsigned int ix = 0; ix < size_x_; ix++)
        {
        unsigned char cost = getCost(ix, iy);
        fprintf(fp, "%d ", cost);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}

}  // namespace costmap_2d
