#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Mesh.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

using namespace tinyxml2;
using namespace std;

Line::Line(int a, int b){
	this->vertexIds[0] = a;
	this->vertexIds[1] = b;
}
bool Scene::visible(double den, double num, double *t_e, double *t_l){
	double t;
	if (den > double(0.0)){
		t =  num / den;
		if (t > *t_l){
			return false;
		}
		if ( t > *t_e){
			*t_e = t;
		}
	}
	else if (den < 0.0){
		t =  num / den;
		if (t < *t_e){
			return false;
		}
		if (t < *t_l){
			*t_l = t;
		}
	}
	else if (num > 0.0){
		return false;
	}
	return true;
	

}
/*
	Transformations, clipping, culling, rasterization are done here.
	You may define helper functions.
*/
void Scene::forwardRenderingPipeline(Camera *camera)
{
	// USE verticesV2 !!!!!!!!!! 
	//verticesV2 -> transformations applied
	vector< Vec3 > verticesV2;
	for(Vec3 *vec : this->vertices){
		Vec3 newVec = Vec3(*vec);
		verticesV2.push_back(newVec);
	}
	
	vector< Vec4 > vec4_with_w;
	for(Vec3 vec : verticesV2){
		Vec4 newVec = Vec4(vec.x, vec.y, vec.z, 1, vec.colorId);
		vec4_with_w.push_back(newVec);
	}
	vector <Line> line_All;

    double matrix_111[4][4] = {double(0.0)};
	matrix_111[0][0] = (double) 1.0;
	matrix_111[1][1] = (double) 1.0;
	matrix_111[2][2] = (double) 1.0;
	matrix_111[3][3] = (double) 1.0;
	Matrix4 finalMatrix = Matrix4(matrix_111);
	// Modeling transformation
	for(Mesh *mesh : this->meshes){
		for (int i = 0 ; i < mesh->numberOfTransformations; i++){
			if(mesh->transformationTypes[i] == 's'){ // Scaling
				Scaling scaling = *this->scalings[mesh->transformationIds[i]-1];
				double matrix[4][4] = {double(0.0)};
				matrix[0][0] = scaling.sx;
				matrix[1][1] = scaling.sy;
				matrix[2][2] = scaling.sz;
				matrix[3][3] = double(1.0);
				Matrix4 matrixV1 = Matrix4(matrix);
				for ( int j = 0 ; j < mesh-> numberOfTriangles ; j++ ){
					// first vertex
					Vec4 vec4_1 = Vec4(this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->x, this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->y,this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->colorId);
					Vec4 tmpVec4_1 =  multiplyMatrixWithVec4(matrixV1, vec4_1);
					verticesV2[mesh->triangles[j].getFirstVertexId() - 1] = Vec3(tmpVec4_1.x, tmpVec4_1.y, tmpVec4_1.z, tmpVec4_1.colorId);
					vec4_with_w[mesh->triangles[j].getFirstVertexId() - 1] = tmpVec4_1; // asli ekledi
					// second vertex
					Vec4 vec4_2 = Vec4(this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->x, this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->y,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->colorId);
					Vec4 tmpVec4_2 =  multiplyMatrixWithVec4(matrixV1, vec4_2);
					verticesV2[mesh->triangles[j].getSecondVertexId() - 1] = Vec3(tmpVec4_2.x, tmpVec4_2.y, tmpVec4_2.z, tmpVec4_2.colorId);
					vec4_with_w[mesh->triangles[j].getSecondVertexId() - 1] = tmpVec4_2;
					// third vertex
					Vec4 vec4_3 = Vec4(this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->x, this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->y,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->colorId);
					Vec4 tmpVec4_3 =  multiplyMatrixWithVec4(matrixV1, vec4_3);
					verticesV2[mesh->triangles[j].getThirdVertexId() - 1] = Vec3(tmpVec4_3.x, tmpVec4_3.y, tmpVec4_3.z, tmpVec4_3.colorId);
					vec4_with_w[mesh->triangles[j].getThirdVertexId() - 1] = tmpVec4_3;
				}
			}
			else if(mesh->transformationTypes[i] == 'r'){ // Rotation
				Rotation rotation = *this->rotations[mesh->transformationIds[i]-1];
				double matrix[4][4] = {double(0.0)};
				double matrix_1[4][4] = {double(0.0)};
				double RxAngle[4][4] = {double(0.0)};
				// calculate v & z using u
				Vec3 v,w;
				double min = 9999999.0;
				if(min > abs(rotation.ux)){
					min = abs(rotation.ux);
				}
				if(min > abs(rotation.uy)){
					min = abs(rotation.uy);
				}
				if(min > abs(rotation.uz)){
					min = abs(rotation.uz);
				}
				if(min == abs(rotation.ux)){
					// calculate for min = ux
					v.x = double(0);
					v.y = -(rotation.uz);
					v.z = rotation.uy;
				}
				else if (min == abs(rotation.uy)){
					// calculate for min = uy
					v.y = double(0);
					v.x = -(rotation.uz);
					v.z = rotation.ux;
				}
				else{
					// calculate for min = uz
					v.z = double(0);
					v.x = -(rotation.uy);
					v.y = rotation.ux;
				}
				Vec3 u = Vec3(rotation.ux, rotation.uy, rotation.uz, -1);
				w = crossProductVec3(u,v);
				w = normalizeVec3(w);
				v = normalizeVec3(v);
				matrix[0][0] = rotation.ux;
				matrix[0][1] = rotation.uy;
				matrix[0][2] = rotation.uz;
				matrix[1][0] = v.x;
				matrix[1][1] = v.y;
				matrix[1][2] = v.z;
				matrix[2][0] = w.x;
				matrix[2][1] = w.y;
				matrix[2][2] = w.z;
				matrix[3][3] = double(1.0);
				
				matrix_1[0][0] =  rotation.ux;
				matrix_1[1][0] =  rotation.uy;
				matrix_1[2][0] =  rotation.uz;
				matrix_1[0][1] =  v.x;
				matrix_1[1][1] =  v.y;
				matrix_1[2][1] =  v.z;
				matrix_1[0][2] =  w.x;
				matrix_1[1][2] =  w.y;
				matrix_1[2][2] =  w.z;
				matrix_1[3][3] =  double(1.0);

				RxAngle[0][0] = double(1.0);
				RxAngle[1][1] = cos(rotation.angle);
				RxAngle[1][2] = -sin(rotation.angle);
				RxAngle[2][1] = sin(rotation.angle);
				RxAngle[2][2] = cos(rotation.angle);
				RxAngle[3][3] = double(1.0);
				Matrix4 rxAngle = Matrix4(RxAngle);
				Matrix4 matrixV1 = Matrix4(matrix);
				Matrix4 matrixV1inverse = Matrix4(matrix_1);
				for (int j = 0; j < mesh->numberOfTriangles; j++)
				{
					Matrix4 finalMatrix = multiplyMatrixWithMatrix(multiplyMatrixWithMatrix(matrixV1inverse, rxAngle), matrixV1);
					//first
					Vec4 vec4First = Vec4(this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->x, this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->y,this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->colorId);
					Vec4 tmpVec4First = multiplyMatrixWithVec4(finalMatrix, vec4First);
					verticesV2[mesh->triangles[j].getFirstVertexId() - 1] = Vec3(tmpVec4First.x, tmpVec4First.y, tmpVec4First.z, tmpVec4First.colorId);
					vec4_with_w[mesh->triangles[j].getFirstVertexId() - 1] = tmpVec4First; 
					//second
					Vec4 vec4Second = Vec4(this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->x, this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->y,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->colorId);
					Vec4 tmpVec4Second = multiplyMatrixWithVec4(finalMatrix, vec4Second);
					verticesV2[mesh->triangles[j].getSecondVertexId() - 1] = Vec3(tmpVec4Second.x, tmpVec4Second.y, tmpVec4Second.z, tmpVec4Second.colorId);
					vec4_with_w[mesh->triangles[j].getSecondVertexId() - 1] = tmpVec4Second;
					//third
					Vec4 vec4Third = Vec4(this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->x, this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->y,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->colorId);
					Vec4 tmpVec4Third = multiplyMatrixWithVec4(finalMatrix, vec4Third);
					verticesV2[mesh->triangles[j].getThirdVertexId() - 1] = Vec3(tmpVec4Third.x, tmpVec4Third.y, tmpVec4Third.z, tmpVec4Third.colorId);
					vec4_with_w[mesh->triangles[j].getThirdVertexId() - 1] = tmpVec4Third;
				}
			}
			else if (mesh->transformationTypes[i] == 't'){ // Translation
				Translation translation = *this->translations[mesh->transformationIds[i]-1];
				double matrix[4][4] = {double(0.0)};
				matrix[0][0] = double(1.0);
				matrix[0][3] = translation.tx;
				matrix[1][1] = double(1.0);
				matrix[1][3] = translation.ty;
				matrix[2][2] = double(1.0);
				matrix[2][3] = translation.tz;
				matrix[3][3] = double(1.0);
				Matrix4 matrixV1 = Matrix4(matrix);
				for ( int j = 0 ; j < mesh-> numberOfTriangles ; j++ ){
					//first
					Vec4 vec4First = Vec4(this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->x, this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->y,this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getFirstVertexId() - 1]->colorId);
					Vec4 tmpVec4First = multiplyMatrixWithVec4(matrixV1, vec4First);
					verticesV2[mesh->triangles[j].getFirstVertexId() - 1] = Vec3(tmpVec4First.x, tmpVec4First.y, tmpVec4First.z, tmpVec4First.colorId);
					vec4_with_w[mesh->triangles[j].getFirstVertexId() - 1] = tmpVec4First;
					//second
					Vec4 vec4Second = Vec4(this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->x, this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->y,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->colorId);
					Vec4 tmpVec4Second = multiplyMatrixWithVec4(matrixV1, vec4Second);
					verticesV2[mesh->triangles[j].getSecondVertexId() - 1] = Vec3(tmpVec4Second.x, tmpVec4Second.y, tmpVec4Second.z, tmpVec4Second.colorId);
					vec4_with_w[mesh->triangles[j].getSecondVertexId() - 1] = tmpVec4Second;
					//third
					Vec4 vec4Third = Vec4(this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->x, this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->y,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->colorId);
					Vec4 tmpVec4Third = multiplyMatrixWithVec4(matrixV1, vec4Third);
					verticesV2[mesh->triangles[j].getThirdVertexId() - 1] = Vec3(tmpVec4Third.x, tmpVec4Third.y, tmpVec4Third.z, tmpVec4Third.colorId);
					vec4_with_w[mesh->triangles[j].getThirdVertexId() - 1] = tmpVec4Third;
				}
			}
		}
	}
	
	// Camera Transformation
	double Mcam[4][4] = {double(0.0)};
	Mcam[0][0] = camera->u.x;
	Mcam[0][1] = camera->u.y;
	Mcam[0][2] = camera->u.z;
	Mcam[1][0] = camera->v.x;
	Mcam[1][1] = camera->v.y;
	Mcam[1][2] = camera->v.z;
	Mcam[2][0] = camera->w.x;
	Mcam[2][1] = camera->w.y;
	Mcam[2][2] = camera->w.z;
	Mcam[0][3] = -(camera->u.x * camera->pos.x + camera->u.y * camera->pos.y + camera->u.z * camera->pos.z);
	Mcam[1][3] = -(camera->v.x * camera->pos.x + camera->v.y * camera->pos.y + camera->v.z * camera->pos.z);
	Mcam[2][3] = -(camera->w.x * camera->pos.x + camera->w.y * camera->pos.y + camera->w.z * camera->pos.z);
	Mcam[3][3] = double(1.0);
	Matrix4 matrixMcam = Matrix4(Mcam);

	for(int j = 0 ; j < verticesV2.size(); j++){
		Vec4 vec4First = Vec4(verticesV2[j].x, verticesV2[j].y, verticesV2[j].z, double(1), verticesV2[j].colorId);
		Vec4 tmpVec4First = multiplyMatrixWithVec4(matrixMcam, vec4First);
		verticesV2[j] = Vec3(tmpVec4First.x, tmpVec4First.y, tmpVec4First.z, tmpVec4First.colorId);
		vec4_with_w[j] = tmpVec4First;
	}

	// Projection Transoformations
	if (camera->projectionType){
		// Perspective Projection
		double Morth[4][4] = {double(0.0)};
		double Mp2o[4][4] = {double(0.0)};
		Morth[0][0] = 2 / (camera->right - camera->left);
		Morth[1][1] = 2 / (camera->top - camera->bottom);
		Morth[2][2] = -2 / (camera->far - camera->near);
		Morth[3][0] = -(camera->right + camera->left) / (camera->right - camera->left);
		Morth[3][1] = -(camera->top + camera->bottom) / (camera->top - camera->bottom);
		Morth[3][2] = -(camera->far + camera->near) / (camera->far - camera->near);
		Morth[3][3] = double(1.0);
		Mp2o[0][0] = camera->near;
		Mp2o[1][1] = camera->near;
		Mp2o[2][2] = camera->far + camera->near;
		Mp2o[2][3] = camera->far * camera->near;
		Mp2o[3][2] = double(-1);
		Matrix4 matrixOrth = Matrix4(Morth);
		Matrix4 matrixP2o = Matrix4(Mp2o);
		for(int j = 0 ; j < verticesV2.size(); j++){
			Vec4 vec4First = Vec4(verticesV2[j].x, verticesV2[j].y,verticesV2[j].z,1,verticesV2[j].colorId);
			Vec4 tmpVec4First = multiplyMatrixWithVec4(multiplyMatrixWithMatrix(matrixOrth,matrixP2o), vec4First);
			verticesV2[j] = Vec3(tmpVec4First.x, tmpVec4First.y, tmpVec4First.z, tmpVec4First.colorId);
			vec4_with_w[j] = tmpVec4First;
		}
	}
	else{
		// Orthographic Projection
		double Morth[4][4] = {double(0.0)};
		Morth[0][0] = 2 / (camera->right - camera->left);
		Morth[1][1] = 2 / (camera->top - camera->bottom);
		Morth[2][2] = -2 / (camera->far - camera->near);
		Morth[3][0] = -(camera->right + camera->left) / (camera->right - camera->left);
		Morth[3][1] = -(camera->top + camera->bottom) / (camera->top - camera->bottom);
		Morth[3][2] = -(camera->far + camera->near) / (camera->far - camera->near);
		Morth[3][3] = double(1.0);
		Matrix4 matrixOrth = Matrix4(Morth);
		for(int j = 0 ; j < verticesV2.size(); j++){
			Vec4 vec4First = Vec4(verticesV2[j].x, verticesV2[j].y,verticesV2[j].z,1,verticesV2[j].colorId);
			Vec4 tmpVec4First = multiplyMatrixWithVec4(matrixOrth, vec4First);
			verticesV2[j] = Vec3(tmpVec4First.x, tmpVec4First.y, tmpVec4First.z, tmpVec4First.colorId);
			vec4_with_w[j] = tmpVec4First;
		}
	}

	// CLIPPING FOR WIREFRAME

	for(Mesh *mesh : this->meshes){
		// FOR WIREFRAME
		if(!(mesh->type)){
			// visible olanlar line_all icerisindeki line'lar. type'ı wireframe olan mesh'lerin ucgenlerini ve vertice'lerini discard et !
			for (int j = 0; j < mesh->numberOfTriangles; j++){
				// For vertex 1 -> 2
				Line tmp = Line(mesh->triangles[j].getFirstVertexId(),mesh->triangles[j].getSecondVertexId());
				double max_w = max(abs(vec4_with_w[tmp.vertexIds[0] - 1].t), abs(vec4_with_w[tmp.vertexIds[1] - 1].t));
				double min_w = -max_w;
				double t_e = 0.0, t_l = 1.0;
				bool visible_variable = false;
				double d_x = vec4_with_w[tmp.vertexIds[1] - 1].x - vec4_with_w[tmp.vertexIds[0]- 1].x;
				double d_y = vec4_with_w[tmp.vertexIds[1] - 1].y - vec4_with_w[tmp.vertexIds[0] - 1].y;
				double d_z = vec4_with_w[tmp.vertexIds[1] - 1].z - vec4_with_w[tmp.vertexIds[0] - 1].z;

				if (visible(d_x, min_w - vec4_with_w[tmp.vertexIds[0]-1].x, &t_e, &t_l)){
					if (visible(-d_x, vec4_with_w[tmp.vertexIds[0]-1].x - max_w, &t_e, &t_l )){
						if (visible(d_y, min_w - vec4_with_w[tmp.vertexIds[0]-1].y, &t_e, &t_l)){
							if (visible(-d_y,vec4_with_w[tmp.vertexIds[0]-1].y - max_w, &t_e, &t_l)){
								if (visible(d_z, min_w - vec4_with_w[tmp.vertexIds[0]-1].z, &t_e, &t_l)){
									if (visible(-d_z, vec4_with_w[tmp.vertexIds[0]-1].z - max_w, &t_e, &t_l)){
										visible_variable = true;
										if (t_l < double(1.0)){
											// TODO: color computation & (if exist) w computation.
											double dst_vertice0 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].z),2));
											double dst_vertice1 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].z),2));
											double tot_dist =  dst_vertice0 + dst_vertice1;
											double tmp_r = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_r += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_r /= tot_dist;

											double tmp_g = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_g += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_g /= tot_dist;

											double tmp_b = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_b += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_b /= tot_dist;

											Color tmp_color = Color(tmp_r, tmp_g, tmp_b);
											this->colorsOfVertices.push_back(&tmp_color);
											
											Vec4 final_1_vec4 = Vec4(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l), vec4_with_w[tmp.vertexIds[1]-1].t, this->colorsOfVertices.size());
											Vec3 final_1_vec3 = Vec3(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l), this->colorsOfVertices.size());
											vec4_with_w.push_back(final_1_vec4);
											verticesV2.push_back(final_1_vec3);
											tmp.vertexIds[1] = verticesV2.size();
										}
										if ( t_e > double(0.0)){
											double dst_vertice0 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].z),2));
											double dst_vertice1 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].z),2));
											double tot_dist =  dst_vertice0 + dst_vertice1;
											double tmp_r = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_r += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_r /= tot_dist;

											double tmp_g = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->g); 
											tmp_g += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->g); 
											tmp_g /= tot_dist;

											double tmp_b = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->b); 
											tmp_b += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->b); 
											tmp_b /= tot_dist;

											Color tmp_color = Color(tmp_r, tmp_g, tmp_b);
											this->colorsOfVertices.push_back(&tmp_color);
											Vec4 final_0_vec4 = Vec4(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e), vec4_with_w[tmp.vertexIds[0]-1].t, this->colorsOfVertices.size());
											Vec3 final_0_vec3 = Vec3(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e), this->colorsOfVertices.size());
											vec4_with_w.push_back(final_0_vec4);
											verticesV2.push_back(final_0_vec3);
											tmp.vertexIds[0] = verticesV2.size();
										}
										line_All.push_back(tmp);
									}
								}
							}
						}
					}
				}
				// For vertex 2 -> 3
				tmp = Line(mesh->triangles[j].getSecondVertexId(),mesh->triangles[j].getThirdVertexId());
				max_w = max(abs(vec4_with_w[tmp.vertexIds[0] - 1].t), abs(vec4_with_w[tmp.vertexIds[1] - 1].t));
				min_w = -max_w;
				t_e = 0.0, t_l = 1.0;
				visible_variable = false;
				d_x = vec4_with_w[tmp.vertexIds[1] - 1].x - vec4_with_w[tmp.vertexIds[0]- 1].x;
				d_y = vec4_with_w[tmp.vertexIds[1] - 1].y - vec4_with_w[tmp.vertexIds[0] - 1].y;
				d_z = vec4_with_w[tmp.vertexIds[1] - 1].z - vec4_with_w[tmp.vertexIds[0] - 1].z;

				if (visible(d_x, min_w - vec4_with_w[tmp.vertexIds[0]-1].x, &t_e, &t_l)){
					if (visible(-d_x, vec4_with_w[tmp.vertexIds[0]-1].x - max_w, &t_e, &t_l )){
						if (visible(d_y, min_w - vec4_with_w[tmp.vertexIds[0]-1].y, &t_e, &t_l)){
							if (visible(-d_y,vec4_with_w[tmp.vertexIds[0]-1].y - max_w, &t_e, &t_l)){
								if (visible(d_z, min_w - vec4_with_w[tmp.vertexIds[0]-1].z, &t_e, &t_l)){
									if (visible(-d_z, vec4_with_w[tmp.vertexIds[0]-1].z - max_w, &t_e, &t_l)){
										visible_variable = true;
										if (t_l < double(1.0)){
											// TODO: color computation & (if exist) w computation.
											double dst_vertice0 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].z),2));
											double dst_vertice1 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].z),2));
											double tot_dist =  dst_vertice0 + dst_vertice1;
											double tmp_r = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_r += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_r /= tot_dist;

											double tmp_g = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_g += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_g /= tot_dist;

											double tmp_b = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_b += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_b /= tot_dist;
											Color tmp_color = Color(tmp_r, tmp_g, tmp_b);
											this->colorsOfVertices.push_back(&tmp_color);
											Vec4 final_1_vec4 = Vec4(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l), vec4_with_w[tmp.vertexIds[0]-1].t, this->colorsOfVertices.size());
											Vec3 final_1_vec3 = Vec3(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l), this->colorsOfVertices.size());
											vec4_with_w.push_back(final_1_vec4);
											verticesV2.push_back(final_1_vec3);
											tmp.vertexIds[1] = verticesV2.size();
										}
										if ( t_e > double(0.0)){
											double dst_vertice0 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].z),2));
											double dst_vertice1 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].z),2));
											double tot_dist =  dst_vertice0 + dst_vertice1;
											double tmp_r = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_r += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_r /= tot_dist;

											double tmp_g = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->g); 
											tmp_g += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->g); 
											tmp_g /= tot_dist;

											double tmp_b = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->b); 
											tmp_b += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->b); 
											tmp_b /= tot_dist;
											Color tmp_color = Color(tmp_r, tmp_g, tmp_b);
											this->colorsOfVertices.push_back(&tmp_color);
											Vec4 final_0_vec4 = Vec4(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e), vec4_with_w[tmp.vertexIds[0]-1].t, this->colorsOfVertices.size());
											Vec3 final_0_vec3 = Vec3(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e), this->colorsOfVertices.size());
											vec4_with_w.push_back(final_0_vec4);
											verticesV2.push_back(final_0_vec3);
											tmp.vertexIds[0] = verticesV2.size();
										}
										line_All.push_back(tmp);
									}
								}
							}
						}
					}
				}
				// For vertex 2 -> 3
				tmp = Line(mesh->triangles[j].getThirdVertexId(),mesh->triangles[j].getFirstVertexId());
				max_w = max(abs(vec4_with_w[tmp.vertexIds[0] - 1].t), abs(vec4_with_w[tmp.vertexIds[1] - 1].t));
				min_w = -max_w;
				t_e = 0.0, t_l = 1.0;
				visible_variable = false;
				d_x = vec4_with_w[tmp.vertexIds[1] - 1].x - vec4_with_w[tmp.vertexIds[0]- 1].x;
				d_y = vec4_with_w[tmp.vertexIds[1] - 1].y - vec4_with_w[tmp.vertexIds[0] - 1].y;
				d_z = vec4_with_w[tmp.vertexIds[1] - 1].z - vec4_with_w[tmp.vertexIds[0] - 1].z;

				if (visible(d_x, min_w - vec4_with_w[tmp.vertexIds[0]-1].x, &t_e, &t_l)){
					if (visible(-d_x, vec4_with_w[tmp.vertexIds[0]-1].x - max_w, &t_e, &t_l )){
						if (visible(d_y, min_w - vec4_with_w[tmp.vertexIds[0]-1].y, &t_e, &t_l)){
							if (visible(-d_y,vec4_with_w[tmp.vertexIds[0]-1].y - max_w, &t_e, &t_l)){
								if (visible(d_z, min_w - vec4_with_w[tmp.vertexIds[0]-1].z, &t_e, &t_l)){
									if (visible(-d_z, vec4_with_w[tmp.vertexIds[0]-1].z - max_w, &t_e, &t_l)){
										visible_variable = true;
										if (t_l < double(1.0)){
											// TODO: color computation & (if exist) w computation.
											double dst_vertice0 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l)) - vec4_with_w[tmp.vertexIds[0]-1].z),2));
											double dst_vertice1 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l)) - vec4_with_w[tmp.vertexIds[1]-1].z),2));
											double tot_dist =  dst_vertice0 + dst_vertice1;
											double tmp_r = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_r += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_r /= tot_dist;

											double tmp_g = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_g += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_g /= tot_dist;

											double tmp_b = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_b += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_b /= tot_dist;
											Color tmp_color = Color(tmp_r, tmp_g, tmp_b);
											this->colorsOfVertices.push_back(&tmp_color);
											Vec4 final_1_vec4 = Vec4(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l), vec4_with_w[tmp.vertexIds[0]-1].t, this->colorsOfVertices.size());
											Vec3 final_1_vec3 = Vec3(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_l), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_l), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_l),  this->colorsOfVertices.size());
											vec4_with_w.push_back(final_1_vec4);
											verticesV2.push_back(final_1_vec3);
											tmp.vertexIds[1] = verticesV2.size();
										}
										if ( t_e > double(0.0)){
											double dst_vertice0 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e)) - vec4_with_w[tmp.vertexIds[0]-1].z),2));
											double dst_vertice1 = sqrt(pow(((vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].x),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].y),2) + pow(((vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e)) - vec4_with_w[tmp.vertexIds[1]-1].z),2));
											double tot_dist =  dst_vertice0 + dst_vertice1;
											double tmp_r = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->r); 
											tmp_r += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->r); 
											tmp_r /= tot_dist;

											double tmp_g = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->g); 
											tmp_g += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->g); 
											tmp_g /= tot_dist;

											double tmp_b = dst_vertice1 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[0]-1].colorId -1]->b); 
											tmp_b += dst_vertice0 * (this->colorsOfVertices[vec4_with_w[tmp.vertexIds[1]-1].colorId -1]->b); 
											tmp_b /= tot_dist;
											Color tmp_color = Color(tmp_r, tmp_g, tmp_b);
											this->colorsOfVertices.push_back(&tmp_color);
											Vec4 final_0_vec4 = Vec4(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e), vec4_with_w[tmp.vertexIds[0]-1].t, this->colorsOfVertices.size());
											Vec3 final_0_vec3 = Vec3(vec4_with_w[tmp.vertexIds[0]-1].x + d_x * (t_e), vec4_with_w[tmp.vertexIds[0]-1].y + d_y * (t_e), vec4_with_w[tmp.vertexIds[0]-1].z + d_z * (t_e), this->colorsOfVertices.size());
											vec4_with_w.push_back(final_0_vec4);
											verticesV2.push_back(final_0_vec3);
											tmp.vertexIds[0] = verticesV2.size();
										}
										line_All.push_back(tmp);
									}
								}
							}
						}
					}
				}
			}
		}
	}

	for(int i = 0;i < verticesV2.size();i++){
		cout << "verticeV2 " << i << ": " << verticesV2[i] << "\n";
		cout << "vertice_with_w " << i << ": " << vec4_with_w[i] << "\n";
	}
	for(Mesh *mesh : this->meshes){
		// FOR WIREFRAME
		if((mesh->type)){
			for (int j = 0; j < mesh->numberOfTriangles; j++){
				// TODO: bu clipping loop'u disinda w division yapmak gerekebilir. alttaki islemler ve wireframe icindeki w division
				// taktik: if t ==1 se bölme. 	OR	 verticeV2[i] == vec4_with_w[i] se w division yap
				// w division for first vertex.
				// TODO: vec4_with_w 'nun x,y,z'sini de t'ye bölmemiz gerekmez mi? sonradan hiç kullanmayacak mıyız?
				if(vec4_with_w[mesh->triangles[j].getFirstVertexId()-1 ].t != double(1.0)){
					verticesV2[mesh->triangles[j].getFirstVertexId()-1 ].x /= vec4_with_w[mesh->triangles[j].getFirstVertexId()-1 ].t;
					verticesV2[mesh->triangles[j].getFirstVertexId()-1 ].y /= vec4_with_w[mesh->triangles[j].getFirstVertexId()-1 ].t;
					verticesV2[mesh->triangles[j].getFirstVertexId()-1 ].z /= vec4_with_w[mesh->triangles[j].getFirstVertexId()-1 ].t;
					vec4_with_w[mesh->triangles[j].getFirstVertexId() - 1].t = double(1.0);
				}
				// w division for second vertex.
				if(vec4_with_w[mesh->triangles[j].getSecondVertexId()-1 ].t != double(1.0)){
					verticesV2[mesh->triangles[j].getSecondVertexId()-1 ].x /= vec4_with_w[mesh->triangles[j].getSecondVertexId()-1 ].t;
					verticesV2[mesh->triangles[j].getSecondVertexId()-1 ].y /= vec4_with_w[mesh->triangles[j].getSecondVertexId()-1 ].t;
					verticesV2[mesh->triangles[j].getSecondVertexId()-1 ].z /= vec4_with_w[mesh->triangles[j].getSecondVertexId()-1 ].t;
					vec4_with_w[mesh->triangles[j].getSecondVertexId() - 1].t = double(1.0);
				}
				// w division for third vertex.
				if(vec4_with_w[mesh->triangles[j].getThirdVertexId()-1 ].t != double(1.0)){
					verticesV2[mesh->triangles[j].getThirdVertexId()-1 ].x /= vec4_with_w[mesh->triangles[j].getThirdVertexId()-1 ].t;
					verticesV2[mesh->triangles[j].getThirdVertexId()-1 ].y /= vec4_with_w[mesh->triangles[j].getThirdVertexId()-1 ].t;
					verticesV2[mesh->triangles[j].getThirdVertexId() -1 ].z /= vec4_with_w[mesh->triangles[j].getThirdVertexId()-1 ].t;
					vec4_with_w[mesh->triangles[j].getThirdVertexId() - 1].t = double(1.0);
				}
			}
		}
	}

	for (int j = 0; j < line_All.size(); j++){ // w division for lines.
		if(vec4_with_w[line_All[j].vertexIds[0] - 1].t != double(1.0)){
		verticesV2[line_All[j].vertexIds[0] - 1].x /= vec4_with_w[line_All[j].vertexIds[0] - 1].t;
		verticesV2[line_All[j].vertexIds[0] - 1].y /= vec4_with_w[line_All[j].vertexIds[0] - 1].t;
		verticesV2[line_All[j].vertexIds[0] - 1].z /= vec4_with_w[line_All[j].vertexIds[0] - 1].t;
		vec4_with_w[line_All[j].vertexIds[0] - 1].t = double(1.0);
		}
		if(vec4_with_w[line_All[j].vertexIds[1] - 1].t != double(1.0)){
		verticesV2[line_All[j].vertexIds[1] - 1].x /= vec4_with_w[line_All[j].vertexIds[1] - 1].t;
		verticesV2[line_All[j].vertexIds[1] - 1].y /= vec4_with_w[line_All[j].vertexIds[1] - 1].t;
		verticesV2[line_All[j].vertexIds[1] - 1].z /= vec4_with_w[line_All[j].vertexIds[1] - 1].t;
		vec4_with_w[line_All[j].vertexIds[1] - 1].t = double(1.0);
		}
	}

	for(int i = 0;i < verticesV2.size();i++){
		cout << "AFTER DIVISION vertice " << i << ": " << verticesV2[i] << "\n";
		cout << "vertice_with_w " << i << ": " << vec4_with_w[i] << "\n";
	}
	// Viewport transformation.
	double viewPort[4][4] = {double(0.0)};
	viewPort[0][0] = camera->horRes / 2;
	viewPort[0][3] = (camera->horRes - 1) / 2;
	viewPort[1][1] = camera->verRes / 2;
	viewPort[1][3] = (camera->verRes - 1) / 2;
	viewPort[2][2] = 0.5;
	viewPort[2][3] = 0.5;
	Matrix4 viewPortMatrix = Matrix4(viewPort);
	for (int j = 0; j < verticesV2.size(); j++)
	{
		Vec4 vec4 = Vec4(verticesV2[j].x, verticesV2[j].y, verticesV2[j].z, double(1.0), verticesV2[j].colorId);
		Vec4 result = multiplyMatrixWithVec4(viewPortMatrix, vec4);
		verticesV2[j] = Vec3(result.x, result.y, result.z, vec4.colorId);
	}

	// for(int i = 0;i < verticesV2.size();i++){
	// 	cout << "vertice " << i << ": " << verticesV2[i] << "\n";
	// }

	// for(int i = 0;i < verticesV2.size();i++){
	// 	cout<< camera->horRes << " " << camera->verRes << "\n";
	// 	cout << "vertice " << i << ": " << verticesV2[i] << "\n";
	// }

	// Rasterization
	for (int j = 0; j < line_All.size(); j++){ // rasterize all lines inserted
		double x0 = verticesV2[line_All[j].vertexIds[0] - 1].x;
		double x1 = verticesV2[line_All[j].vertexIds[1] - 1].x;
		double y0 = verticesV2[line_All[j].vertexIds[0] - 1].y;
		double y1 = verticesV2[line_All[j].vertexIds[1] - 1].y;
		double y = y0;
		double d = (y0 - y1) + 0.5 * (x1 - x0);
		Color *c = this->colorsOfVertices[verticesV2[line_All[j].vertexIds[0] - 1].colorId -1];
		Color *c1 = this->colorsOfVertices[verticesV2[line_All[j].vertexIds[1] - 1].colorId - 1];
		Color dc = Color();
		dc.r = (c1->r - c->r) / abs(x1 - x0);
		dc.g = (c1->g - c->g) / abs(x1 - x0);
		dc.b = (c1->b - c->b) / abs(x1 - x0);
		if (x1 > x0)
		{
			for (int x = x0; x <= x1; x++){
				this->image[x][y].r = c->r;
				this->image[x][y].g = c->g;
				this->image[x][y].b = c->b;
				if(d < 0){
					y += 1;
					d += (y0 - y1) + (x1 - x0);
				}
				else{
					d += y0 - y1;
				}
				c->r += dc.r;
				c->g += dc.g;
				c->b += dc.b;
			}
		}
		else{
			for (int x = x1; x <= x0; x++){
				this->image[x][y].r = c->r;
				this->image[x][y].g = c->g;
				this->image[x][y].b = c->b;
				if(d < 0){
					y += 1;
					d += (y0 - y1) + (x1 - x0);
				}
				else{
					d += y0 - y1;
				}
				c->r += dc.r;
				c->g += dc.g;
				c->b += dc.b;
			}
		}
	}

	for(Mesh *mesh : this->meshes){ //rasterization for solids
		if(mesh->type){
			for (int j = 0; j < mesh->numberOfTriangles; j++){
				double x0 = verticesV2[mesh->triangles[j].getFirstVertexId() - 1].x;
				double y0 = verticesV2[mesh->triangles[j].getFirstVertexId() - 1].y;
				double x1 = verticesV2[mesh->triangles[j].getSecondVertexId() - 1].x;
				double y1 = verticesV2[mesh->triangles[j].getSecondVertexId() - 1].y;
				double x2 = verticesV2[mesh->triangles[j].getThirdVertexId() - 1].x;
				double y2 = verticesV2[mesh->triangles[j].getThirdVertexId() - 1].y;

				// for (int y = min({y0, y1, y2}); y <= max({y0, y1, y2}; y++)){
					
				// }
			}
		}
	}
	
}


/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL) {
		str = pElement->GetText();
		
		if (strcmp(str, "enabled") == 0) {
			cullingEnabled = true;
		}
		else {
			cullingEnabled = false;
		}
	}

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		// read projection type
		str = pCamera->Attribute("type");

		if (strcmp(str, "orthographic") == 0) {
			cam->projectionType = 0;
		}
		else {
			cam->projectionType = 1;
		}

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read meshes
	pElement = pRoot->FirstChildElement("Meshes");

	XMLElement *pMesh = pElement->FirstChildElement("Mesh");
	XMLElement *meshElement;
	while (pMesh != NULL)
	{
		Mesh *mesh = new Mesh();

		pMesh->QueryIntAttribute("id", &mesh->meshId);

		// read projection type
		str = pMesh->Attribute("type");

		if (strcmp(str, "wireframe") == 0) {
			mesh->type = 0;
		}
		else {
			mesh->type = 1;
		}

		// read mesh transformations
		XMLElement *pTransformations = pMesh->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			mesh->transformationTypes.push_back(transformationType);
			mesh->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		mesh->numberOfTransformations = mesh->transformationIds.size();

		// read mesh faces
		char *row;
		char *clone_str;
		int v1, v2, v3;
		XMLElement *pFaces = pMesh->FirstChildElement("Faces");
        str = pFaces->GetText();
		clone_str = strdup(str);

		row = strtok(clone_str, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);
			
			if (result != EOF) {
				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		meshes.push_back(mesh);

		pMesh = pMesh->NextSiblingElement("Mesh");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}
