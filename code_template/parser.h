#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>

namespace parser
{

    struct Vec3i
    {
        int x, y, z;
    };

    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    struct Vec3f
    {
        float x, y, z;

        Vec3f(){}
        Vec3f(float x, float y, float z): x(x), y(y), z(z){}
        Vec3f(const Vec3f &vec): x(vec.x), y(vec.y), z(vec.z){}
        Vec3f(const Vec3i &vec): x(vec.x), y(vec.y), z(vec.z){}

        Vec3f operator* (float scl) const{
            return Vec3f(
                    x*scl,
                    y*scl,
                    z*scl
                    );
        }

        Vec3f operator* (const Vec3f &vec) const{
            return Vec3f(
                    x*vec.x,
                    y*vec.y,
                    z*vec.z
            );
        }
        Vec3f operator+ (const Vec3f &vec){
            return {
                    x + vec.x,
                    y + vec.y,
                    z + vec.z
                    };
        }

        Vec3f operator+= (const Vec3f& vec) {
            x += vec.x;
            y += vec.y;
            z += vec.z;
            return *this;
        }

        Vec3f operator- (const Vec3f &vec) const{
            return {
                    x - vec.x,
                    y - vec.y,
                    z - vec.z
            };
        }

        Vec3f operator/ (float scl) const{
            return {
                    x/scl,
                    y/scl,
                    z/scl
            };
        }

        Vec3f operator/ (Vec3f vec) const{
            return {
                    x/vec.x,
                    y/vec.y,
                    z/vec.z
            };
        }


        Vec3f operator= (const Vec3f &vec){
            return {
                    x = vec.x,
                    y = vec.y,
                    z = vec.z
            };
        }

    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        bool is_mirror;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
    };

    struct Triangle
    {
        int material_id;
        Face indices;

        Triangle(){}
        Triangle(int id, Face inds): material_id(id), indices(inds){}

    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        //Functions
        void loadFromXml(const std::string &filepath);
    };
}

#endif
