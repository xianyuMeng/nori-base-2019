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

#include <nori/accel.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
}

void Accel::build() {
    /* Nothing to do here for now */
}
bool Accel::rayIntersect(
    const Ray3f& ray_,
    Intersection& its,
    bool shadowRay,
    OctreeBaseNode* root
) const
{
    bool foundIntersection = false;
    uint32_t f = (uint32_t) -1;
    Ray3f ray(ray_);

    OctreeBaseNode* node = root;
    std::stack<OctreeBaseNode*> dfs_stack;
    std::vector<std::pair<OctreeBaseNode*, float>> leaf_nodes;
    dfs_stack.push(node);

//    float curT = std::numeric_limits<float>::max();

    while(!dfs_stack.empty())
    {
        auto top = dfs_stack.top();
        dfs_stack.pop();
        if(checkLeaf(top) && top->m_triangle_idx.size() > 0)
        {
            float tt1, tt2;
            top->m_bbox.rayIntersect(ray, tt1, tt2);
            leaf_nodes.push_back(std::make_pair(top, tt1));
        }
        for(size_t i = 0; i < 8; ++i)
        {
            if(top->children[i])
            {
                if(top->children[i]->m_bbox.rayIntersect(ray))
                {
                    dfs_stack.push(top->children[i]);
                }
            }
        }
    }
    //while(!dfs_stack.empty())
    //{
    //    auto top = dfs_stack.top();
    //    dfs_stack.pop(); 
    //    if(top->m_bbox.rayIntersect(ray))
    //    {
    //        if(checkLeaf(top) && top->m_triangle_idx.size() > 0)
    //        {
    //            leaf_nodes.push_back(top);
    //        } 
    //    }

    //    for(size_t i = 0; i < 8; ++i)
    //    {
    //        if(top->children[i])
    //        {
    //            dfs_stack.push(top->children[i]);
    //        }
    //    }
    //}

    struct{
        bool operator()(const std::pair<OctreeBaseNode*, float>& a, const std::pair<OctreeBaseNode*, float>& b) const
        {
            return a < b;
        }
    }leafSort;
    bool flag = false;
    if(!leaf_nodes.empty())
    {
        std::sort(leaf_nodes.begin(), leaf_nodes.end(), leafSort);
        while(!flag)
        {
            for(size_t i = 0; i < leaf_nodes.size(); ++i)
            {
                for(size_t j = 0; j < leaf_nodes[i].first->m_triangle_idx.size(); ++j)
                {
                    float u, v, t;
                    if(m_mesh->rayIntersect(leaf_nodes[i].first->m_triangle_idx[j], ray, u, v, t))
                    {
                        if(shadowRay) return true;
                        ray.maxt = its.t = t;
                        its.uv = Point2f(u, v);
                        its.mesh = m_mesh;
                        f = leaf_nodes[i].first->m_triangle_idx[j];
                        foundIntersection = true;
                        flag = true;
                        fprintf(stdout, "leaf %d; tri %d; tri id %d; t %f, box t %f;\n", i, j, f, t, leaf_nodes[i].second); 
                        break;
                    }
                }
            }
            fprintf(stdout, "\t\n");
            flag = true;
        }
   }
    //if(!leaf_nodes.empty())
    //{
    //    std::sort(leaf_nodes.begin(), leaf_nodes.end(), leafSort); 
    //    for(size_t j = 0; j < leaf_nodes[0].first->m_triangle_idx.size(); ++j)
    //    {
    //        float u, v, t;
    //        if(m_mesh->rayIntersect(leaf_nodes[0].first->m_triangle_idx[j], ray, u, v, t))
    //        {
    //            if(shadowRay) return true;
    //            ray.maxt = its.t = t;
    //            its.uv = Point2f(u, v);
    //            its.mesh = m_mesh;
    //            f = leaf_nodes[0].first->m_triangle_idx[j];
    //            foundIntersection = true;
    //        }

    //    }
    
    //}
   //for(size_t i = 0; i < leaf_nodes.size(); ++i)
    //{
    //    for(size_t j = 0; j < leaf_nodes[i]->m_triangle_idx.size(); ++j)
    //    {
    //        float u, v, t;
    //        if(m_mesh->rayIntersect(leaf_nodes[i]->m_triangle_idx[j], ray, u, v, t))
    //        {
    //            if(shadowRay) return true;
    //            ray.maxt = its.t = t;
    //            its.uv = Point2f(u, v);
    //            its.mesh = m_mesh;
    //            f = leaf_nodes[i]->m_triangle_idx[j];
    //            foundIntersection = true;
    //        }
    //    }
    //} 
    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    /* Brute force search through all triangles */
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        float u, v, t;
        if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            if (shadowRay)
                return true;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_mesh;
            f = idx;
            foundIntersection = true;
        }
    }

    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

NORI_NAMESPACE_END

