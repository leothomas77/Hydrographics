#define PATH_SIZE 256

class ModelData {
public:
  ModelData(char *path, Vec3 translation, float scale, Vec3 rotation) :
    translation{ translation }, realSize{ scale }, rotation{ rotation }
  {
    if (strlen(path) > (PATH_SIZE - 1))
      throw std::runtime_error(std::string("Model name too long!"));
    strcpy(this->path, path);
  }
  char path[PATH_SIZE];
  Vec3 translation = Vec3(0.0f);
  float realSize = 0.0f;// default value
  Vec3 rotation;
};
