#pragma once

#include <string>
#include <sstream>
#include <vector>

class IniParser
{
public:
	IniParser( );
	~IniParser( );

	bool Init( std::string strFile );
	bool IsFileUpdated();
	bool ParseConfig( std::string strAppName , std::string strKeyName , std::string& Output );
	bool ParseConfig( std::string strAppName , std::string strKeyName , bool& Output );
	bool ParseConfig( std::string strAppName , std::string strKeyName , int& Output );
	bool ParseConfig(std::string strAppName, std::string strKeyName, unsigned int& Output);
	bool ParseConfig(std::string strAppName, std::string strKeyName, float& Output);
	bool ParseConfig( std::string strAppName , std::string strKeyName , double& Output );
	bool ParseConfig(std::string strAppName, std::string strKeyName, std::vector<double>& Output);
	bool ParseConfig(std::string strAppName, std::string strKeyName, std::vector<std::vector<double>>& Output);
	
private:
	const static int FILE_PATH_BUFF_SIZE = 255;
	std::string m_strCfgFile;
	time_t m_fileUpdateTime=-1;
	time_t m_fileInitTime=-1;	
	bool ExtractArrayValueFromString(std::string strArray, std::vector<double>& vecValues);

};

class IniExporter
{
public:
	IniExporter();
	~IniExporter();

	bool Init(std::string strFile);
	bool ExportConfig(std::string strAppName, std::string strKeyName, std::string Val);
	bool ExportConfig(std::string strAppName, std::string strKeyName, bool Val);
	bool ExportConfig(std::string strAppName, std::string strKeyName, int Val);
	bool ExportConfig(std::string strAppName, std::string strKeyName, unsigned int Val);
	bool ExportConfig(std::string strAppName, std::string strKeyName, float Val);
	bool ExportConfig(std::string strAppName, std::string strKeyName, double Val, unsigned int nPrecision = 18);
	bool ExportConfig(std::string strAppName, std::string strKeyName, std::vector<double> Val);
	bool ExportConfig(std::string strAppName, std::string strKeyName, std::vector<std::vector<double>> Val);

private:
	const static int FILE_PATH_BUFF_SIZE = 255;
	std::string m_strCfgFile;

private:
	std::string getDoubleToString(double value, unsigned int nPrecision);
};
