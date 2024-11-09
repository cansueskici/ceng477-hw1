#include <iostream>
#include <cmath>
#include "parser.h"
#include "ppm.h"

using namespace parser;
typedef unsigned char RGB[3];

/*
 * for each pixel s
 * compute r // done
 * make t_min inf, obj null // done
 *      foreach obj o
 *          if r intersects o
 *              if t < t_min:
 *                  t_min = t, obj = o
 *      if obj not null
 *      pixel_color = L_a
 *      foreach light l:
 *          compute shadow ray s (from x to l)
 *          foreach object p
 *              if s intersects p before the light source
 *                  continue // in shadow!
 *          pixel_color += L_d + L_s
 *      else pixel_color = bg_color
 *
 * */

/*
 *
 * structs
 *
 * */

struct Ray{
    Vec3f origin;
    Vec3f direction;

    Ray(){}
    Ray(Vec3f org, Vec3f dir): origin(org), direction(dir){}

};

struct Intersection {

    bool intersects;
    int materialId;
    float tValue;
    Vec3f surfaceNormal;
    Vec3f intPoint;
    Vec3f color;

    Intersection():intersects(false), tValue(std::numeric_limits<float>::max()){}
    Intersection(bool intcs, int mat_id, float tval, Vec3f nrml, Vec3f intpt)
    :intersects(intcs), materialId(mat_id), tValue(tval), surfaceNormal(nrml), intPoint(intpt){}

};

/*
 *
 * math calculations
 *
 **/

float calcDeterminant(const Vec3f &v0, const Vec3f &v1, const Vec3f &v2){
    return v0.x * (v1.y*v2.z - v2.y*v1.z)
           + v0.y * (v2.x*v1.z - v1.x*v2.z)
           + v0.z * (v1.x*v2.y - v1.y*v2.x);
}


const float dotProduct(Vec3f &v1, Vec3f &v2){
    return (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
}

Vec3f normalize(Vec3f vec) {
    Vec3f newVec;
    float len = sqrt(dotProduct(vec, vec));
    newVec.x = vec.x/len;
    newVec.y = vec.y/len;
    newVec.z = vec.z/len;
    return newVec;
}

Vec3f crossProduct(Vec3f &v1, Vec3f &v2){
    return Vec3f(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);

}

const float calcDistance(Vec3f &v1, Vec3f &v2){
    return sqrt(pow((v1.x - v2.x),2) + pow((v1.y - v2.y),2) + pow((v1.z - v2.z),2));
}

Vec3f clamp(Vec3f color){
    color.x = std::min(255.0f, std::max(0.0f, color.x));
    color.y = std::min(255.0f, std::max(0.0f, color.y));
    color.z = std::min(255.0f, std::max(0.0f, color.z));

    return color;
}

Vec3i clamp(Vec3i color){
    color.x = std::min(255, std::max(0, color.x));
    color.y = std::min(255, std::max(0, color.y));
    color.z = std::min(255, std::max(0, color.z));
    return color;
}

/*
 *
 * ray calculations
 *
 * */

Ray getRay(int w, int h, Camera &cam){

    float l = cam.near_plane.x;
    float r = cam.near_plane.y;
    float b = cam.near_plane.z;
    float t = cam.near_plane.w;

    float s_u = (r - l)*(w+0.5)/cam.image_width;
    float s_v = (t- b)*(h+0.5)/cam.image_height;

    Vec3f u = normalize(crossProduct(cam.gaze, cam.up));
    Vec3f gaze_normalized = normalize(cam.gaze);
    Vec3f v = normalize(crossProduct(u, gaze_normalized));

    Vec3f m = cam.position+(gaze_normalized*cam.near_distance);
    Vec3f q = m + (u*l) + (v*t);
    Vec3f s = q + (u*s_u) - (v*s_v);
    Vec3f d = normalize(s - cam.position);

    Ray ray(cam.position, d);
    return ray;
}

Vec3f intersectionPt(Ray &ray, float t){
    // r(t) = o +td

    Vec3f p = ray.origin + ray.direction*t;
    return p;
}

/*
 *
 * Intersections
 *
 * */

Intersection sphereIntersection(Ray &ray, Scene &scene, const Sphere &sphere){
    Intersection pt;

    Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1];
    Vec3f o_c = ray.origin-center;

    float a = dotProduct(ray.direction, ray.direction);
    float b = 2*dotProduct(ray.direction, o_c) ;
    float c = dotProduct(o_c, o_c) - (sphere.radius*sphere.radius);

    float discriminant = (b*b) - 4*(a*c);

    if (discriminant >= 0){

        float t1 = (-b - sqrt(discriminant))/(2*a);
        float t2 = (-b + sqrt(discriminant))/(2*a);
        float t;

        if(t1 < t2){
            t = t1;
        } else{
            t = t2;
        }

        if(t > 0){
            Vec3f intersect_point = intersectionPt(ray, t);
            Vec3f normal_nom = intersect_point - center;
            Vec3f normal = normal_nom/sphere.radius;
            return Intersection(true, sphere.material_id, t, normal, intersect_point);
        }
    }
    return pt;
}

Intersection triangleIntersection(Ray &ray, Scene &scene, const Triangle &triangle){

    Intersection pt;

    Vec3f &a = scene.vertex_data[triangle.indices.v0_id -1];
    Vec3f &b = scene.vertex_data[triangle.indices.v1_id -1];
    Vec3f &c = scene.vertex_data[triangle.indices.v2_id -1];

    Vec3f ab = a - b, ac = a - c, ao = a - ray.origin;

    float beta_determinant = calcDeterminant(ao, ac, ray.direction);
    float gamma_determinant = calcDeterminant(ab, ao, ray.direction);
    float t_determinant = calcDeterminant(ab, ac, ao);
    float a_determinant = calcDeterminant(ab, ac, ray.direction);

    if (a_determinant == 0){
        return pt;
    }

    float t = t_determinant/a_determinant;
    if (t <= 0){
        return pt;
    }

    float gamma = gamma_determinant/a_determinant;
    if (gamma < 0 || gamma > 1){
        return pt;
    }

    float beta = beta_determinant/a_determinant;
    if(beta < 0 || beta > (1-gamma)){
        return pt;
    }

    Vec3f ints = intersectionPt(ray, t);
    Vec3f normal = normalize(crossProduct(ab, ac));

    return Intersection(true, triangle.material_id, t, normal, ints);
}

Intersection meshIntersection(Ray &ray, Scene &scene, const Mesh &mesh){
    Intersection closest;
    int meshSize = (mesh.faces.size());

    for (int i = 0; i < meshSize; ++i) {
        Intersection poss_int = triangleIntersection(ray, scene, Triangle(mesh.material_id, mesh.faces[i]));
        if (poss_int.intersects && poss_int.tValue < closest.tValue){
            closest = poss_int;
        }
    }

    return closest;
}

/*
 *
 * shading
 *
 * */

Vec3f irradiance(PointLight &light, Vec3f &point){
    Vec3f result;
    Vec3f i = light.position - point;
    float d = dotProduct(i, i);

    if(round(d) != 0){
        result =  light.intensity / d;
    }

    return result;
}

Vec3f diffuseShading(PointLight &light, Scene &scene, Intersection &intersection){
    Vec3f l = normalize((light.position - intersection.intPoint));
    float p = dotProduct(l, intersection.surfaceNormal);
    if (p < 0){
        p = 0;
    }

    Vec3f irrd = irradiance(light, intersection.intPoint);

    return scene.materials[intersection.materialId-1].diffuse*(irrd*p);

}

Vec3f specularShading(PointLight &light, Scene &scene, Intersection &intersection, Ray &ray){
    float phong= scene.materials[intersection.materialId-1].phong_exponent;
    Vec3f l = normalize(light.position - intersection.intPoint);
    Vec3f r = normalize(l - ray.direction);

    float p = dotProduct(intersection.surfaceNormal, r);
    if(p < 0){
        p = 0;
    }

    Vec3f irrd = irradiance(light, intersection.intPoint);

    return scene.materials[intersection.materialId-1].specular * irrd * pow(p, phong);
}

/*
 *
 * main handler
 *
 */

Intersection hitFunction(Ray &ray, Scene &scene, Camera &cam, int mirrorRecDepth){

    Intersection result;


    if (mirrorRecDepth < 0){
        return result;
    }

    for (const Sphere &sph : scene.spheres) {
        Intersection closestSphere = sphereIntersection(ray, scene, sph);
        if (closestSphere.intersects && closestSphere.tValue < result.tValue){result = closestSphere;}
    }

    for(const Triangle &tri : scene.triangles){
        Intersection closestTriangle = triangleIntersection(ray, scene, tri);
        if (closestTriangle.intersects && closestTriangle.tValue < result.tValue){result = closestTriangle;}
    }

    for (const Mesh &msh : scene.meshes){
        Intersection closestMesh = meshIntersection(ray, scene, msh);
        if(closestMesh.intersects && closestMesh.tValue < result.tValue){result = closestMesh;}

    }

    if(result.intersects){
        //check for shadow

        Material material = scene.materials[result.materialId-1];

        Vec3f color = material.ambient * scene.ambient_light;

        for (PointLight &light : scene.point_lights){
            bool inShadow = false;

            Vec3f w = normalize(light.position - result.intPoint);
            Vec3f e = w * scene.shadow_ray_epsilon;

            Ray shadowRay(result.intPoint + e, w);

            //t value for point lights, if it is lower than shadow's, object is in light, otherwise in shadow.
            Vec3f plightVec = (light.position - shadowRay.origin)/shadowRay.direction;
            float tLight = plightVec.x;

            for(const Sphere &sph : scene.spheres){
                Intersection sphInts = sphereIntersection(shadowRay, scene, sph);
                if(sphInts.intersects && sphInts.tValue < tLight){inShadow = true;}
            }

            for(const Triangle &tri : scene.triangles){
                Intersection triInts = triangleIntersection(shadowRay, scene, tri);
                if(triInts.intersects && triInts.tValue < tLight){inShadow = true;}
            }

            if(!inShadow){
                for(const Mesh &msh : scene.meshes){
                    Intersection meshInts = meshIntersection(shadowRay, scene, msh);
                    if(meshInts.intersects && meshInts.tValue < tLight){inShadow = true;}
                }
            }

            float distance = calcDistance(light.position, cam.position);
            if(!inShadow || ((distance == 0) && inShadow)){
                Vec3f diffuse = diffuseShading(light, scene, result);
                Vec3f specular = specularShading(light, scene, result, ray);
                Vec3f diffusePlusSpecular = diffuse+specular;
                color += diffusePlusSpecular;

            }
        }

        if(material.is_mirror){
            float dotProductDoubled = 2.0* dotProduct(ray.direction, result.surfaceNormal);
            Vec3f nrml = result.surfaceNormal*dotProductDoubled;
            Vec3f reflectionDirection = normalize(ray.direction - nrml);
            Vec3f eps = reflectionDirection * scene.shadow_ray_epsilon;

            Ray reflection(result.intPoint+eps, reflectionDirection);
            Intersection reflectionIntersection= hitFunction(reflection, scene, cam, mirrorRecDepth - 1);

            if (reflectionIntersection.intersects){color += reflectionIntersection.color * scene.materials[result.materialId-1].mirror;}
        }
        result.color = color;
    }

    return result;

}


int main(int argc, char* argv[]) {
    // Sample usage for reading an XML scene file
    parser::Scene scene;
    scene.loadFromXml(argv[1]);

    for (Camera &cam: scene.cameras) {
        const int width = cam.image_width, height = cam.image_height;
        unsigned char *image = new unsigned char[width * height * 3];
        int i = 0;

        for (int h = 0; h < cam.image_height; h++) {
            for (int w = 0; w < cam.image_width; w++, i+=3) {

                Ray ray = getRay(w, h, cam);
                //find intersection
                Intersection intersection = hitFunction(ray, scene, cam, scene.max_recursion_depth);

                //calculate rgb and pls clamp.
                if (intersection.intersects) {
                    intersection.color = clamp(intersection.color);
                    image[i] = intersection.color.x;
                    image[i+1] = intersection.color.y;
                    image[i+2] = intersection.color.z;
                } else {
                    scene.background_color = clamp(scene.background_color);
                    image[i] = scene.background_color.x;
                    image[i+1] = scene.background_color.y;
                    image[i+2] = scene.background_color.z;
                }
            }
        }
        write_ppm(cam.image_name.c_str(), image, width, height);
    }
    return 0;
}