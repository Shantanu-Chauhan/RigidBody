#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H
#include<string>
#include<unordered_map>
class Texture;
class ResourceManager
{
public:
	ResourceManager();
	~ResourceManager();
	Texture *LoadTexture(const char *pFilePath);
private:
	std::unordered_map<std::string, Texture *> mTextures;
};

#endif // !RESOURCE_MANAGER_H
