/****************************************************************************
*   ����:  jasitzhang(����)
*   �޸ģ� ������  2016-07-28
*   ����:  2011-10-2
*   Ŀ��:  ��ȡ�����ļ�����Ϣ����map����ʽ����
*   Ҫ��:  �����ļ��ĸ�ʽ����#��Ϊ��ע�ͣ����õ���ʽ��key = value���м���пո�Ҳ��û�пո�
*****************************************************************************/
#ifndef _GET_CONFIG_H_
#define _GET_CONFIG_H_

#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <stdlib.h>

using namespace std;

#define COMMENT_CHAR '#'

bool IsSpace(char c)
{
	if (' ' == c || '\t' == c)
		return true;
	return false;
}

bool IsCommentChar(char c)
{
	switch (c) 
	{
	case COMMENT_CHAR://COMMENT_CHAR is '#'
		return true;
	default:
		return false;
	}
}

void Trim(string & str)
{
	if (str.empty()) 
		return;
	
	int i, start_pos, end_pos;
	for (i = 0; i < str.size(); ++i) 
	{
		if (!IsSpace(str[i])) 
			break;
	}
	if (i == str.size()) 
	{ // ȫ���ǿհ��ַ���
		str = "";
		return;
	}

	start_pos = i;

	for (i = str.size() - 1; i >= 0; --i)
	{
		if (!IsSpace(str[i])) 
			break;
	}
	end_pos = i;

	str = str.substr(start_pos, end_pos - start_pos + 1);
}

bool AnalyseLine(const string & line, string & key, string & value)
{
	if (line.empty())
		return false;
	int start_pos = 0, end_pos = line.size() - 1, pos;
	if ((pos = line.find(COMMENT_CHAR)) != -1) 
	{
		if (0 == pos) 
		{  // �еĵ�һ���ַ�����ע���ַ�
			return false;
		}
		end_pos = pos - 1;
	}
	string new_line = line.substr(start_pos, start_pos + 1 - end_pos);  // Ԥ����ɾ��ע�Ͳ���

	if ((pos = new_line.find('=')) == -1)
		return false;  // û��=��

	key = new_line.substr(0, pos);
	value = new_line.substr(pos + 1, end_pos + 1 - (pos + 1));

	Trim(key);
	if (key.empty()) 
		return false;
	
	Trim(value);
	return true;
}

bool ReadConfig(const string & filename, map<string, string> & m)
{
	m.clear();
	ifstream infile(filename.c_str());
	if (!infile) 
	{
		cout << "file open error" << endl;
		return false;
	}
	string line, key, value;
	while (getline(infile, line))
	{
		if (AnalyseLine(line, key, value)) 
			m[key] = value;
	}

	infile.close();
	return true;
}

/*
bool WriteConfig(const string & filename, map<string, string> & m)
{
	ofstream outfile(filename.c_str());
	if (!outfile)
	{
		cout << "config file write error" << endl;
		return false;
	}

	for (auto mite = m.begin(); mite != m.end(); ++mite)
	{
		outfile << mite->first << "=" << mite->second << endl;

	}

	outfile.close();
	return true;
}
*/

void PrintConfig(const map<string, string> & m)
{
	map<string, string>::const_iterator mite = m.begin();
	for (; mite != m.end(); ++mite) 
	{
		cout << mite->first << "=" << mite->second << endl;
	}
}

#endif

// ʹ��

// map<string, string> config;
// bool read = ReadConfig("video.cfg", config);	// ��video.cfg����config��ֵ�ԣ��ɹ�����true
// int red = atoi(config["RED"].c_str());		// �Ѽ�RED��Ӧ��ֵתΪint

// map<string, string> m1;
// m1.insert(pair<string, string>("hi", "hello"));
// WriteConfig("video.cfg", m1);				// ע�����������е����ݣ�Ȼ��д��hi=hello
