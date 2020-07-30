
#ifndef GET_FILE_EXTENSION_H
#define GET_FILE_EXTENSION_H

namespace sensor_fusion
{
	std::string getFileExt(const std::string& path)
	{
		size_t i = path.rfind('.', path.length());
           if (i != std::string::npos)                 
              {
		std::string ext=(path.substr(i+1, path.length() - i));
                return ext;
              }
            return("");
	}

} //namespace sensor_fusion

#endif //GET_FILE_EXTENSION_H
