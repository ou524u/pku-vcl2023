#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

#include <map>
#include <utility>

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const& input, Engine::SurfaceMesh& output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (!G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v = G.Vertex(i); // v is a vertexProxy. how to visit v's position?
                // auto prev_posi = prev_mesh.Positions[i]; // vertex v's position!

                auto neighbors = v->Neighbors(); // returning vertexIdx vector
                // your code here:
                int n_neightbors = neighbors.size();
                float u = 3.0f / 16;
                if (n_neightbors != 3) {
                    u = 3.0f / (8 * n_neightbors);
                }
                auto renewed_posi = (1 - n_neightbors * u) * prev_mesh.Positions[i];
                for (int iv = 0;iv < n_neightbors;iv++) {
                    renewed_posi += u * prev_mesh.Positions[neighbors[iv]];
                }
                curr_mesh.Positions.push_back(renewed_posi);

                // YOU CAN'T USE THE BELOW !!!
                // curr_mesh.Positions[i] = renewed_posi;
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (!eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    float u_bound = 1.0f / 2;
                    auto newed_vertex = (prev_mesh.Positions[e->To()] + prev_mesh.Positions[e->From()]) * u_bound;
                    curr_mesh.Positions.push_back(newed_vertex);
                }
                else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    float u_next = 3.0f / 8;
                    float u_opposite = 1.0f / 8;
                    auto newed_vertex = (prev_mesh.Positions[e->To()] + prev_mesh.Positions[e->From()]) * u_next + (prev_mesh.Positions[e->OppositeVertex()] + prev_mesh.Positions[e->TwinOppositeVertex()]) * u_opposite;
                    curr_mesh.Positions.push_back(newed_vertex);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0 = prev_mesh.Indices[i + 0U];
                auto v1 = prev_mesh.Indices[i + 1U];
                auto v2 = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    {v0, m2, m1},  // Sub-face 1: v0, m0, m2
                    {m2, m0, m1},  // Sub-face 2: m0, v1, m1
                    {m2, v1, m0},  // Sub-face 3: m2, m1, v2
                    {m0, v2, m1}   // Sub-face 4: m0, m1, m2
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t*>(toInsert),
                    reinterpret_cast<std::uint32_t*>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const& input, Engine::SurfaceMesh& output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2{ 0 });

        // Build DCEL.
        DCEL G(input);
        if (!G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        std::vector<DCEL::VertexIdx> boundVertex_vec;

        int cnt = 0;
        for (uint32_t i = 0;i < input.Positions.size();i++) {
            if (G.Vertex(i)->OnBoundary()) {
                if (cnt == 0) {
                    boundVertex_vec.push_back(i);
                    boundVertex_vec.push_back(G.Vertex(i)->BoundaryNeighbors().first);
                }
                cnt++;
            }
        }
        // boundVertex_vec.reserve(cnt * sizeof(DCEL::VertexIdx));
        while (boundVertex_vec.size() < cnt) {
            auto vertex_candidate = G.Vertex(boundVertex_vec[boundVertex_vec.size() - 1])->BoundaryNeighbors();
            if (vertex_candidate.first != boundVertex_vec[boundVertex_vec.size() - 2]) {
                boundVertex_vec.push_back(vertex_candidate.first);
            }
            else {
                boundVertex_vec.push_back(vertex_candidate.second);
                assert(vertex_candidate.second != boundVertex_vec[boundVertex_vec.size() - 2]);
            }
        }
        assert(boundVertex_vec.size() == cnt);


        // Square initialize
        float uvlen = 4.0f / boundVertex_vec.size();
        float uvcurr = 0.0f;
        for (int i = 0;i < boundVertex_vec.size();i++) {
            if (uvcurr >= 0 && uvcurr < 1) {
                output.TexCoords[boundVertex_vec[i]] = (glm::vec2{ 0.0f,uvcurr });
            }
            else if (uvcurr >= 1 && uvcurr < 2) {
                output.TexCoords[boundVertex_vec[i]] = (glm::vec2{ uvcurr - 1.0f,1.0f });
            }
            else if (uvcurr >= 2 && uvcurr < 3) {
                output.TexCoords[boundVertex_vec[i]] = (glm::vec2{ 1.0f,3.0f - uvcurr });
            }
            else {
                output.TexCoords[boundVertex_vec[i]] = (glm::vec2{ 4.0f - uvcurr,0.0f });
            }
            uvcurr += uvlen;
        }
        std::set<DCEL::VertexIdx> boundVertex_set(boundVertex_vec.begin(), boundVertex_vec.end());

        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            for (uint32_t i = 0;i < output.TexCoords.size();i++) {

                // boundaries shall not be changed
                if (boundVertex_set.find(i) != boundVertex_set.end()) { continue; }
                float u = 1.0f / G.Vertex(i)->Neighbors().size();
                auto newuv = glm::vec2{ 0 };

                // j belones to /omega_i
                for (uint32_t j = 0;j < G.Vertex(i)->Neighbors().size();j++) {
                    newuv += u * output.TexCoords[G.Vertex(i)->Neighbors()[j]];
                }
                output.TexCoords[i] = newuv;
            }
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const& input, Engine::SurfaceMesh& output, float simplification_ratio) {

        DCEL G(input);
        if (!G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (!G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Q matrix of the face f.
        auto UpdateQ{
            [&G, &output](DCEL::Triangle const* f) -> glm::mat4 {

                // your code here:

                auto point0 = output.Positions[f->VertexIndex(0)];
                auto point1 = output.Positions[f->VertexIndex(1)];
                auto point2 = output.Positions[f->VertexIndex(2)];

                glm::vec3 V1 = point1 - point0;
                glm::vec3 V2 = point2 - point0;

                glm::vec3 normal = glm::cross(V1, V2);
                normal /= std::sqrt(glm::dot(normal, normal));

                auto A = normal.x;
                auto B = normal.y;
                auto C = normal.z;
                auto D = -(A * point0.x + B * point0.y + C * point0.z);

                glm::mat4 Q{
                    A * A, A * B, A * C, A * D,
                    A * B, B * B, B * C, B * D,
                    A * C, B * C, C * C, C * D,
                    A * D, B * D, C * D, D * D };

                return Q;

            }
        };

        // The struct to record constraction info.
        struct ConstractionPair {
            DCEL::HalfEdge const* edge;            // which edge to constract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$

        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ConstractionPair struct.
        static constexpr auto MakePair{
            [](DCEL::HalfEdge const* edge,
                glm::vec3 const& p1,
                glm::vec3 const& p2,
                glm::mat4 const& Q
            ) -> ConstractionPair {

                glm::mat4 calcQ{
                    glm::vec4(Q[0][0], Q[1][0], Q[2][0], 0.0f),
                    glm::vec4(Q[1][0], Q[1][1], Q[2][1], 0.0f),
                    glm::vec4(Q[2][0], Q[2][1], Q[2][2], 0.0f),
                    glm::vec4(Q[3][0], Q[3][1], Q[3][2], 1.0f) };


                float detQ = glm::determinant(calcQ);

                glm::vec4 targetP{0.0f};
                float costP;
                if (std::fabs(detQ) < 0.001) {
                    // attempt to find the optimal vertex along the segment v1v2.
                    float alpha = 0; // alpha*x1 + (1-alpha)*x2
                    float xy = 2 * p2.x * p2.y - p2.x * p1.y - p1.x * p2.y;
                    float xz = 2 * p2.x * p2.z - p2.x * p1.z - p1.x * p2.z;
                    float yz = 2 * p2.y * p2.z - p2.y * p1.z - p1.y * p2.z;
                    float delx = p2.x - p1.x;
                    float dely = p2.y - p1.y;
                    float delz = p2.z - p1.z;

                    float alpha_b = Q[1][1] * delx * p2.x + Q[2][2] * dely * p2.y + Q[3][3] * delz * p2.z
                        + Q[1][4] * delx + Q[2][4] * dely + Q[3][4] * delz
                        + Q[1][2] * xy + Q[1][3] * xz + Q[2][3] * yz;

                    float alpha_k = 2 * Q[1][2] * delx * dely + 2 * Q[1][3] * delx * delz + 2 * Q[2][3] * dely * delz
                        + Q[1][1] * delx * delx + Q[2][2] * dely * dely + Q[3][3] * delz * delz;

                    alpha = -alpha_b / alpha_k;
                    if (alpha >= 0 && alpha <= 1) {
                        glm::vec4 target_onedge{ alpha * p1 + (1 - alpha) * p2, 1.0f };
                        targetP = target_onedge;
                        costP = glm::dot(targetP, Q * targetP);
                    }
                    // fall back on choosing Nv from amongst the endpoints and the midpoint
                    else {
                        glm::vec4 pointA{ p1, 1.0f };
                        glm::vec4 pointB{ p2, 1.0f };
                        glm::vec4 pointMid = (pointA + pointB) / 2.0f;
                        float costA = glm::dot(pointA, Q * pointA);
                        float costB = glm::dot(pointB, Q * pointB);
                        float costMid = glm::dot(pointMid, Q * pointMid);
                        if (costA < costB) {
                            if (costA < costMid) {
                                costP = costA;
                                targetP = pointA;
                            }
                            else {
                                costP = costMid;
                                targetP = pointMid;
                            }
                        }
                        else {
                            if (costB < costMid) {
                                costP = costB;
                                targetP = pointB;
                            }
                            else {
                                costP = costMid;
                                targetP = pointMid;
                            }
                        }
                    }

                    // NOTE: it's not DIRECTLY chosing directly the midpoint!
                }

                else {
                    targetP = glm::inverse(calcQ) * glm::vec4{ 0.0f, 0.0f, 0.0f, 1.0f };

                    costP = glm::dot(targetP, Q * targetP);
                }
                return { edge, targetP, costP };


            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ConstractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Qf:       $Qf[idx]$ is the Q matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map;
        std::vector<ConstractionPair>                  pairs;
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Qf(G.NumOfFaces(), glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Qf[G.IndexOf(f)] = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the constractable edges.
        for (auto e : G.Edges()) {
            if (!G.IsConstractable(e)) continue;
            auto v1 = e->From();
            auto v2 = e->To();
            auto pair = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)] = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the constractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (!pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsConstractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the constractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the constract result
            // ring:   the edge ring of vertex v1
            ConstractionPair& top = pairs[min_idx];
            auto               v1 = top.edge->From();
            auto               v2 = top.edge->To();
            auto               result = G.Constract(top.edge);
            auto               ring = G.Vertex(v1)->Ring();

            top.edge = nullptr;            // The constraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed] = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Qf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Qf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            std::set<DCEL::VertexIdx> neoVertex_set;
            neoVertex_set.insert(v1);

            Qv[v1] = glm::mat4(0.0f);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Q matrix for $e->Face()$.
                auto new_Qface = UpdateQ(e->Face());
                //     2. According to the difference between the old Q (in $Qf$) and the new Q (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                Qv[e->From()] += (new_Qface - Qf[G.IndexOf(e->Face())]);
                Qv[e->To()] += (new_Qface - Qf[G.IndexOf(e->Face())]);

                neoVertex_set.insert(e->From());
                neoVertex_set.insert(e->To());
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                Qv[v1] += new_Qface;
                //     4. Update $Qf$.
                Qf[G.IndexOf(e->Face())] = new_Qface;
            }

            // Finally, as the Q matrix changed, we should update the relative $ConstractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:

            // simply go through the pairs would be ok.
            // this is not efficient
            for (int i = 0;i < pairs.size();i++) {
                auto e = pairs[i].edge;
                if (e == nullptr) {
                    continue;
                }
                if (neoVertex_set.find(e->From()) == neoVertex_set.end() && neoVertex_set.find(e->To()) == neoVertex_set.end()) {
                    continue;
                }
                else {
                    auto neopair = MakePair(e, output.Positions[e->From()], output.Positions[e->To()], Qv[e->From()] + Qv[e->To()]);
                    // pair_map[G.IndexOf(e)];
                    pairs[i] = neopair;
                }
            }

        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (!G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const& input, Engine::SurfaceMesh& output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent{
            [](glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                glm::vec3 e1 = v1 - vAngle;
                glm::vec3 e2 = v2 - vAngle;

                float cotangent = 0;
                float dotProduct = glm::dot(e1, e2);
                float crossProduct = glm::length(glm::cross(e1, e2));

                // Handle cases where the crossProduct is close to zero (avoid division by zero)
                if (std::fabs(crossProduct) < 0.1) {
                    // Choose an appropriate value or handle the case as needed
                    if (crossProduct > 0) {
                        cotangent = 1000 * dotProduct;
                    }
                    else {
                        cotangent = -1000 * dotProduct;
                    }
                }

                cotangent = dotProduct / crossProduct;

                return cotangent;
            }

        };

        DCEL G(input);

        auto GetSumCotangent{
            [&G](Engine::SurfaceMesh const& pos_mesh, DCEL::HalfEdge const* e) -> float {
                float wij = GetCotangent(pos_mesh.Positions[e->OppositeVertex()], pos_mesh.Positions[e->From()], pos_mesh.Positions[e->To()]);
                float wji = GetCotangent(pos_mesh.Positions[e->TwinEdge()->OppositeVertex()], pos_mesh.Positions[e->To()], pos_mesh.Positions[e->From()]);
                return wij + wji;
            }
        };


        if (!G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (!G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
                glm::vec3 sum_wv{ 0 };
                float sum_w = 0;
                for (auto e : G.Vertex(i)->Ring()) {
                    DCEL::VertexIdx neighbor = e->PrevEdge()->To();
                    if (useUniformWeight) {
                        sum_w += 1;
                        sum_wv += prev_mesh.Positions[neighbor];
                    }
                    else {
                        float wij = GetSumCotangent(prev_mesh, e->PrevEdge());
                        sum_w += wij;
                        sum_wv += wij * prev_mesh.Positions[neighbor];
                    }
                }
                curr_mesh.Positions[i] = (1 - lambda) * prev_mesh.Positions[i] + lambda * (sum_wv / sum_w);
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    // constexpr int EdgeVertexIndex[12][2] = {
    //     {0,1},{2,3} {4,5},{6,7},{0,2},{4,6},{1,3},{5,7},{0,4},{1,5},{2,6},{3,7}
    // };



/******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh& output, const std::function<float(const glm::vec3&)>& sdf, const glm::vec3& grid_min, const float dx, const int n) {
        // your code here:
        struct comp
        {
            bool operator()(const std::pair<glm::ivec3, glm::ivec3>& lpair, const std::pair<glm::ivec3, glm::ivec3>& rpair) const {
                if (lpair.first.x < rpair.first.x) { return true; }
                if (lpair.first.y < rpair.first.y) { return true; }
                if (lpair.first.z < rpair.first.z) { return true; }
                if (lpair.second.x < rpair.second.x) { return true; }
                if (lpair.second.y < rpair.second.y) { return true; }
                return lpair.second.z < rpair.second.z;
            }
        };
        std::map<std::pair<glm::ivec3, glm::ivec3>, uint32_t, comp> vertexs;

        uint32_t vertexcnt = 0;
        glm::ivec3 unit[3]{ {1,0,0},{0,1,0},{0,0,1} };


        for (int ix = 0;ix < n;++ix) {
            for (int iy = 0;iy < n;++iy) {
                for (int iz = 0;iz < n;++iz) {

                    glm::vec3 vzero_pos = grid_min + dx * glm::vec3{ ix,iy,iz };
                    glm::vec3 v_pos[8];
                    float v_sdf[8];
                    uint8_t v_condition = 0;
                    for (int iv = 0;iv < 8;++iv) {
                        v_pos[iv] = vzero_pos + glm::vec3{ (iv & 1) * dx,((iv >> 1) & 1) * dx,(iv >> 2) * dx };
                        v_sdf[iv] = sdf(v_pos[iv]);
                        if (v_sdf[iv] > 0) {
                            v_condition += (1 << iv);
                        }
                    }
                    uint16_t e_condition = c_EdgeStateTable[v_condition];
                    uint32_t iv_onedge[12]; // to count vertexindexes on each edge
                    for (int ie = 0;ie < 12;++ie) { // loop through edge
                        if (!((e_condition >> ie) & 1)) {
                            iv_onedge[ie] = -1;
                            continue;
                        }
                        // get the vertex from&to, expressed in 2 ways: vec3 and local_vertex_index 
                        glm::ivec3 ie_from = (ie & 1) * unit[((ie >> 2) + 1) % 3] + ((ie >> 1) & 1) * unit[((ie >> 2) + 2) % 3];
                        glm::ivec3 ie_to = ie_from + unit[(ie >> 2)];

                        int v_from = ie_from.x + ie_from.y * 2 + ie_from.z * 4;
                        int v_to = ie_to.x + ie_to.y * 2 + ie_to.z * 4;

                        auto ie_vpair = std::make_pair(glm::ivec3{ ix,iy,iz } + ie_from, glm::ivec3{ ix,iy,iz } + ie_to);

                        if (vertexs.find(ie_vpair) == vertexs.end()) {
                            // vertex not initiated before
                            vertexs.insert(std::make_pair(ie_vpair, vertexcnt));
                            iv_onedge[ie] = vertexcnt;
                            // calculate vertex position, and push_back

                            glm::vec3 vonedge_pos = (std::fabs(v_sdf[v_to]) * v_pos[v_from] + std::fabs(v_sdf[v_from]) * v_pos[v_to])
                                / (std::fabs(v_sdf[v_from]) + std::fabs(v_sdf[v_to]));

                            output.Positions.push_back(vonedge_pos);

                            vertexcnt++;
                        }
                        else {
                            iv_onedge[ie] = vertexs.at(ie_vpair);
                            // no need to calculate vertex position
                        }
                    }
                    // after looping through edge, get iv_onedge[12]
                    for (int k = 0;k < 5;++k) {
                        int e0 = c_EdgeOrdsTable[v_condition][3 * k];
                        int e1 = c_EdgeOrdsTable[v_condition][3 * k + 1];
                        int e2 = c_EdgeOrdsTable[v_condition][3 * k + 2];
                        if (e0 >= 0 && e1 >= 0 && e2 >= 0) {
                            // assert(iv_onedge[e0] >= 0 && iv_onedge[e1] >= 0 && iv_onedge[e2] >= 0);
                            // there was a trans:int->uint32_t
                            output.Indices.push_back(iv_onedge[e2]);
                            output.Indices.push_back(iv_onedge[e1]);
                            output.Indices.push_back(iv_onedge[e0]);

                        }
                    }

                }
            }
        }
        return;

    }
} // namespace VCX::Labs::GeometryProcessing
