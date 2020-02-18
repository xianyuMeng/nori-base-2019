/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <nori/mesh.h>
#include <nori/octreenode.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Acceleration data structure for ray intersection queries
 *
 * The current implementation falls back to a brute force loop
 * through the geometry.
 */
class Accel {
public:
    /**
     * \brief Register a triangle mesh for inclusion in the acceleration
     * data structure
     *
     * This function can only be used before \ref build() is called
     */
    void addMesh(Mesh *mesh);

    /// Build the acceleration data structure (currently a no-op)
    void build();

    unsigned int MIN_TRI = 20;
    int total_leaf = 0;
    int total_interior = 0; 
    
    std::vector<int> triangle_idx;

    void init_triangle_idx(size_t size) 
    {
        for(size_t i = 0; i < size; ++i)
        {
            this->triangle_idx.push_back(i); 
        }
    }
    OctreeBaseNode* build(const BoundingBox3f& bbox, std::vector<int>& triangle_idx, int* leaf, int* interior)
    {
        fprintf(stdout, "leaf %d; node %d\n", (*leaf), (*interior));
        if(triangle_idx.size() == 0)
        {
            return nullptr;
        }
        if(triangle_idx.size() < MIN_TRI)
        {
            OctreeBaseNode* node = new OctreeLeaf(bbox, triangle_idx);
            (*leaf)++;
            return node;
        }

        std::vector<int> triangle_list[8];
        BoundingBox3f sub_bbox[8];
        calSubBox(bbox, sub_bbox);

        for(size_t i = 0; i < triangle_idx.size(); ++i)
        {
            BoundingBox3f box = m_mesh->getBoundingBox(triangle_idx[i]);
            for(size_t j = 0; j < 8; ++j)
            {
                if(box.overlaps(sub_bbox[j]))
                {
                    triangle_list[j].push_back(triangle_idx[i]);
                }
            }
        }       
        (*interior)++;
        OctreeBaseNode* node = new OctreeNode(bbox);
        for(size_t i = 0; i < 8; ++i)
        {
            node->children[i] = this->build(sub_bbox[i], triangle_list[i], leaf, interior);
        }
        return node;
    }

    void calSubBox(const BoundingBox3f& bbox, BoundingBox3f* sub_bbox) const
    {
        Vector3f len = bbox.getExtents();
        len /= 2.0;
        Point3f min = bbox.min;
        Point3f max = bbox.max;
        sub_bbox[0] = BoundingBox3f(Point3f(min[0], min[1], min[2]), Point3f(min[0] + len[0], min[1] + len[1], min[2] + len[2]));
        sub_bbox[1] = BoundingBox3f(Point3f(min[0] + len[0], min[1], min[2]), Point3f(max[0], min[1] + len[1], min[2] + len[2]));
        sub_bbox[2] = BoundingBox3f(Point3f(min[0] + len[0], min[1] + len[1], min[2]), Point3f(max[0], max[1], min[2] + len[2]));
        sub_bbox[3] = BoundingBox3f(Point3f(min[0], min[1] + len[1], min[2]), Point3f(min[0] + len[0], max[1], min[2] + len[2]));
        sub_bbox[4] = BoundingBox3f(Point3f(min[0], min[1], min[2] + len[2]), Point3f(min[0] + len[0], min[1] + len[1], max[2]));
        sub_bbox[5] = BoundingBox3f(Point3f(min[0] + len[0], min[1], min[2] + len[2]), Point3f(max[0], min[1] + len[1], max[2]));
        sub_bbox[6] = BoundingBox3f(Point3f(min[0] + len[0], min[1] + len[1], min[2] + len[2]), Point3f(max[0], max[1], max[2]));
        sub_bbox[7] = BoundingBox3f(Point3f(min[0], min[1] + len[1], min[2] + len[2]), Point3f(min[0] + len[0], max[1], max[2]));
    }

    /// Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }
    int getTriCount() const { return m_mesh->getTriangleCount();}

    /**
     * \brief Intersect a ray against all triangles stored in the scene and
     * return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum extent
     *    information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \param shadowRay
     *    \c true if this is a shadow ray query, i.e. a query that only aims to
     *    find out whether the ray is blocked or not without returning detailed
     *    intersection information.
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;

private:
    Mesh         *m_mesh = nullptr; ///< Mesh (only a single one for now)
    BoundingBox3f m_bbox;           ///< Bounding box of the entire scene
};

NORI_NAMESPACE_END
