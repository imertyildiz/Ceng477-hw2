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
					// second vertex
					Vec4 vec4_2 = Vec4(this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->x, this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->y,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->colorId);
					Vec4 tmpVec4_2 =  multiplyMatrixWithVec4(matrixV1, vec4_2);
					verticesV2[mesh->triangles[j].getSecondVertexId() - 1] = Vec3(tmpVec4_2.x, tmpVec4_2.y, tmpVec4_2.z, tmpVec4_2.colorId);
					// third vertex
					Vec4 vec4_3 = Vec4(this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->x, this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->y,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->colorId);
					Vec4 tmpVec4_3 =  multiplyMatrixWithVec4(matrixV1, vec4_3);
					verticesV2[mesh->triangles[j].getThirdVertexId() - 1] = Vec3(tmpVec4_3.x, tmpVec4_3.y, tmpVec4_3.z, tmpVec4_3.colorId);
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
					//second
					Vec4 vec4Second = Vec4(this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->x, this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->y,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->colorId);
					Vec4 tmpVec4Second = multiplyMatrixWithVec4(finalMatrix, vec4Second);
					verticesV2[mesh->triangles[j].getSecondVertexId() - 1] = Vec3(tmpVec4Second.x, tmpVec4Second.y, tmpVec4Second.z, tmpVec4Second.colorId);
					//third
					Vec4 vec4Third = Vec4(this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->x, this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->y,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->colorId);
					Vec4 tmpVec4Third = multiplyMatrixWithVec4(finalMatrix, vec4Third);
					verticesV2[mesh->triangles[j].getThirdVertexId() - 1] = Vec3(tmpVec4Third.x, tmpVec4Third.y, tmpVec4Third.z, tmpVec4Third.colorId);
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
					//second
					Vec4 vec4Second = Vec4(this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->x, this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->y,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getSecondVertexId() - 1]->colorId);
					Vec4 tmpVec4Second = multiplyMatrixWithVec4(matrixV1, vec4Second);
					verticesV2[mesh->triangles[j].getSecondVertexId() - 1] = Vec3(tmpVec4Second.x, tmpVec4Second.y, tmpVec4Second.z, tmpVec4Second.colorId);
					//third
					Vec4 vec4Third = Vec4(this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->x, this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->y,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->z,1,this->vertices[mesh->triangles[j].getThirdVertexId() - 1]->colorId);
					Vec4 tmpVec4Third = multiplyMatrixWithVec4(matrixV1, vec4Third);
					verticesV2[mesh->triangles[j].getThirdVertexId() - 1] = Vec3(tmpVec4Third.x, tmpVec4Third.y, tmpVec4Third.z, tmpVec4Third.colorId);
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
	Matrix4 matrixMcam = Matrix4(Mcam);

	for(int j = 0 ; j < verticesV2.size(); j++){
		Vec4 vec4First = Vec4(verticesV2[j].x, verticesV2[j].y, verticesV2[j].z, double(1), verticesV2[j].colorId);
		Vec4 tmpVec4First = multiplyMatrixWithVec4(matrixMcam, vec4First);
		verticesV2[j] = Vec3(tmpVec4First.x, tmpVec4First.y, tmpVec4First.z, tmpVec4First.colorId);
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
			for (int j = 0; j < mesh->numberOfTriangles; j++){
				line_All.push_back(Line(mesh->triangles[j].getFirstVertexId(),mesh->triangles[j].getSecondVertexId()));
				line_All.push_back(Line(mesh->triangles[j].getSecondVertexId(),mesh->triangles[j].getThirdVertexId()));
				line_All.push_back(Line(mesh->triangles[j].getThirdVertexId(),mesh->triangles[j].getFirstVertexId()));
			}
		}
	}




	// Triangle rasterization
	for (int j = 0; j < camera->verRes; j++){
		for (int i = 0; i < camera->horRes; i++){
			// alpha = f12(i,j) / f12(x0,y0)
			double alpha;
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