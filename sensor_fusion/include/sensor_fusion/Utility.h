#ifndef SENSOR_FUSION_UTILITY_H
#define SENSOR_FUSION_UTILITY_H

#include <string>
#include <sstream>
#include <optional>
namespace Utility
{

	template <typename DataType, typename = std::enable_if_t<!std::is_pointer<DataType>::value> >
	std::optional<DataType> StringToDataType(const std::string& input)
	{
		DataType result;
		std::stringstream field_convert{input};
		field_convert >> std::noskipws >>result;
		
		if ((field_convert.rdstate() & std::ifstream::failbit) != 0)
		{
			return std::nullopt;
		}
		return result;
	}
	
	template<char delimiter>
	class WordDelimitedBy : public std::string
	{};

	template <char delimiter>
	std::istream& operator>>(std::istream& is, WordDelimitedBy<delimiter>& output)
	{
		std::getline(is, output, delimiter);
		return is;
	}

} //namespace Utility

#endif //SENSOR_FUSION_UTILITY_H
