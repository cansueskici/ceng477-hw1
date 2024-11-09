#include <iostream>
#include <math.h>
#include "parser.h"
#include "ppm.h"

using namespace parser;
typedef unsigned char RGB[3];

enum object_type{NONE, SPH, TRI, MESH};
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
    object_type obj_type;
    Vec3f surfaceNormal;
    Vec3f intPoint;
    Vec3f color;

    Intersection():intersects(false){}
    Intersection(bool itsct): intersects(itsct){}
    Intersection(bool intcs, int mat_id, float tval, object_type obty, Vec3f nrml, Vec3f intpt)
    :intersects(intcs), materialId(mat_id), tValue(tval), obj_type(obty), surfaceNormal(nrml), intPoint(intpt){}

};

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
//    if (color.x > 255) color.x = 255; else if (color.x < 0) color.x = 0; else color.x = round(color.x);
//    if (color.y > 255) color.y = 255; else if (color.y < 0) color.y = 0; else color.y = round(color.y);
//    if (color.z > 255) color.z = 255; else if (color.z < 0) color.z = 0; else color.z = round(color.z);
//    if (color.x > 255) color.x = 255; else if (color.x < 0) color.x = 0; else color.x = color.x;
//    if (color.y > 255) color.y = 255; else if (color.y < 0) color.y = 0; else color.y = color.y;
//    if (color.z > 255) color.z = 255; else if (color.z < 0) color.z = 0; else color.z = color.z;
    return color;

}

Vec3i clamp(Vec3i color){
    color.x = std::min(255, std::max(0, color.x));
    color.y = std::min(255, std::max(0, color.y));
    color.z = std::min(255, std::max(0, color.z));
//    if (color.x > 255) color.x = 255; else if (color.x < 0) color.x = 0; else color.x = round(color.x);
//    if (color.y > 255) color.y = 255; else if (color.y < 0) color.y = 0; else color.y = round(color.y);
//    if (color.z > 255) color.z = 255; else if (color.z < 0) color.z = 0; else color.z = round(color.z);
//    if (color.x > 255) color.x = 255; else if (color.x < 0) color.x = 0; else color.x = color.x;
//    if (color.y > 255) color.y = 255; else if (color.y < 0) color.y = 0; else color.y = color.y;
//    if (color.z > 255) color.z = 255; else if (color.z < 0) color.z = 0; else color.z = color.z;
    return color;

}



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

Vec3f intersectionPt(Ray &ray, double t){
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
        /*float firstTerm = (-1)*b/2;
        float secondTerm = sqrt(pow(b/2, 2) - a*(dotProduct(o_c, o_c))- pow(sphere.radius,2));

        float t1 = (firstTerm + secondTerm)/a;
        float t2 = (firstTerm - secondTerm)/a;
*/
        float t1 = (-b - sqrt(discriminant))/(2*a);
        float t2 = (-b + sqrt(discriminant))/(2*a);

        float t = (t1 < t2) ? t1: t2;
        if(t > 0){
            Vec3f intersect_point = intersectionPt(ray, t);
            Vec3f normal = (intersect_point - center)/sphere.radius;
            return Intersection(true, sphere.material_id, t, SPH, normal, intersect_point);
        }
    }
    return pt;
}

Intersection triangleIntersection(Ray &ray, Scene &scene, const Triangle &triangle){

    Intersection pt;

    Vec3f &a = scene.vertex_data[triangle.indices.v0_id -1];
    Vec3f &b = scene.vertex_data[triangle.indices.v1_id -1];
    Vec3f &c = scene.vertex_data[triangle.indices.v2_id -1];

    Vec3f ab = a - b;
    Vec3f ac = a - c;
    Vec3f ao = a - ray.origin;

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
    Vec3f temp = crossProduct(ray.direction, ac);
    float az = dotProduct(ab, temp);
    if ( az > -1e-6 && az <1e-6){
        return pt;
    }



    Vec3f ints = ray.origin + ray.direction*t;
    Vec3f ba = b-a;
    Vec3f ca = c-a;
    Vec3f normal = normalize(crossProduct(ba, ca));
    return Intersection(true, triangle.material_id, t, TRI, normal, ints);
}

Intersection meshIntersection(Ray &ray, Scene &scene, const Mesh &mesh){
    Intersection closest;
    closest.tValue = std::numeric_limits<float>::max();
    for (const Face &face: mesh.faces){
        Intersection possible_intersection = triangleIntersection(ray,scene, Triangle(mesh.material_id, face));
        if(possible_intersection.intersects && possible_intersection.tValue < closest.tValue){
            closest = possible_intersection;
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
    Vec3f i = light.position - point;
    float d = dotProduct(i, i);
    if(round(d) != 0){
        return light.intensity / d;
    }
    return Vec3f();

}

Vec3f diffuseShading(PointLight &light, Scene &scene, Intersection &intersection){
    Vec3f irrd = irradiance(light, intersection.intPoint);
    Vec3f l = normalize((light.position - intersection.intPoint));
    float  p = dotProduct(l, intersection.surfaceNormal) >= 0 ? dotProduct(l, intersection.surfaceNormal): 0;
    return scene.materials[intersection.materialId-1].diffuse*(irrd*p);

}

Vec3f specularShading(PointLight &light, Scene &scene, Intersection &intersection, Ray &ray){
    Vec3f irrd = irradiance(light, intersection.intPoint);
    Material material = scene.materials[intersection.materialId-1];
    Vec3f l = normalize(light.position - intersection.intPoint);
    Vec3f r = normalize(l - ray.direction);

    float p = dotProduct(intersection.surfaceNormal, r) >=0? dotProduct(intersection.surfaceNormal, r): 0;
    return material.specular * irrd * pow(p, material.phong_exponent);
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
    result.tValue = std::numeric_limits<float>::max();

    for (int i = 0; i < scene.spheres.size(); ++i) {
        const Sphere &sph = scene.spheres[i];
        Intersection closestSphere = sphereIntersection(ray, scene, sph);
        if (closestSphere.intersects && closestSphere.tValue < result.tValue){
            result = closestSphere;
        }
    }

    for (int j = 0; j < scene.triangles.size(); ++j) {
        const Triangle &tri = scene.triangles[j];
        Intersection closestTriangle = triangleIntersection(ray,scene,tri);
        if (closestTriangle.intersects && closestTriangle.tValue < result.tValue){
            result = closestTriangle;
        }
    }

    for (int k = 0; k < scene.meshes.size(); ++k) {
        const Mesh &msh = scene.meshes[k];
        Intersection closestMesh = meshIntersection(ray, scene, msh);
        if (closestMesh.intersects && closestMesh.tValue < result.tValue){
            result = closestMesh;
        }
    }

    if(result.intersects){

        //check for shadow
        Material material = scene.materials[result.materialId-1];
        Vec3f color = material.ambient * scene.ambient_light;

        for (PointLight &light : scene.point_lights){
            bool inShadow = false;
            float distance = calcDistance(light.position, cam.position);

            Vec3f w = normalize(light.position - result.intPoint);
            Vec3f e = w * scene.shadow_ray_epsilon;

            Ray shadowRay(result.intPoint + e, w);

            //t value for point lights, if it is lower than shadow's, object is in light, otherwise in shadow.

            float tLight = (light.position - shadowRay.origin).x/shadowRay.direction.x;

            for (int i = 0; i < scene.spheres.size(); ++i) {
                const Sphere &sph = scene.spheres[i];
                Intersection sphInts = sphereIntersection(shadowRay, scene, sph);
                if(sphInts.intersects && sphInts.tValue < tLight){
                    inShadow = true;
                }
            }

            for (int j = 0; j < scene.triangles.size(); ++j) {
                const Triangle &tri = scene.triangles[j];
                Intersection triInts = triangleIntersection(shadowRay, scene, tri);
                if (triInts.intersects && triInts.tValue < tLight){
                    inShadow = true;
                }
            }

            if(!inShadow){
                for (int k = 0; k < scene.meshes.size(); ++k) {
                    const Mesh &msh = scene.meshes[k];
                    Intersection meshInts = meshIntersection(shadowRay, scene, msh);
                    if (meshInts.intersects && meshInts.tValue < tLight){
                        inShadow = true;
                    }

                }
            }

            if(!inShadow || (inShadow && distance == 0)){
                int material_id = result.materialId;
                Vec3f diffuse = diffuseShading(light, scene, result);
                Vec3f specular = specularShading(light, scene, result, ray);
                color += diffuse + specular;

            }
        }

        if(material.is_mirror){

            Vec3f reflectionDirection = normalize(ray.direction - result.surfaceNormal * 2.0 * dotProduct(ray.direction, result.surfaceNormal));
            Vec3f eps = reflectionDirection * scene.shadow_ray_epsilon;
            Ray reflection(result.intPoint+eps, reflectionDirection);
            Intersection reflectionIntersection= hitFunction(reflection, scene, cam, mirrorRecDepth - 1);

            if (reflectionIntersection.intersects){
                color += reflectionIntersection.color * scene.materials[result.materialId-1].mirror;
            }
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
            if((h % 100) == 0){
                std::cout << "pirt" << std::endl;

            }
            for (int w = 0; w < cam.image_width; w++, i+=3) {

                Ray ray = getRay(w, h, cam);
                Intersection intersection = hitFunction(ray, scene, cam, scene.max_recursion_depth);
                //find intersection
                //calculate rgb, pls clamp.

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