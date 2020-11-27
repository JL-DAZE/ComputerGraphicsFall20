/*
Copyright (c) 2019, Michael Kazhdan
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#ifndef EXCEPTIONS_INCLUDED
#define EXCEPTIONS_INCLUDED

#include <stdarg.h>
#include <exception>
#include <string>
#include <cstring>

#define VERBOSE_MESSAGING

namespace Util
{
#ifdef VERBOSE_MESSAGING
	inline char *MakeMessageString( const char *header , const char *fileName , int line , const char *functionName , const char *format , ... )
	{
		va_list args;
		va_start( args , format );

		// Formatting is:
		// <header> <filename> (Line <line>)
		// <header size> <function name>
		// <header size> <format message>
		char lineBuffer[25];
		sprintf( lineBuffer , "(Line %d)" , line );
		size_t _size , size=0;

		// Line 1
		size += strlen(header)+1;
		size += strlen(fileName)+1;
		size += strlen(lineBuffer)+1;

		// Line 2
		size += strlen(header)+1;
		size += strlen(functionName)+1;

		// Line 3
		size += strlen(header)+1;
		size += vsnprintf( NULL , 0 , format , args );

		char *_buffer , *buffer = new char[ size+1 ];
		_size = size , _buffer = buffer;

		// Line 1
		sprintf( _buffer , "%s " , header );
		_buffer += strlen(header)+1;
		_size -= strlen(header)+1;

		sprintf( _buffer , "%s " , fileName );
		_buffer += strlen(fileName)+1;
		_size -= strlen(fileName)+1;

		sprintf( _buffer , "%s\n" , lineBuffer );
		_buffer += strlen(lineBuffer)+1;
		_size -= strlen(lineBuffer)+1;

		// Line 2
		for( int i=0 ; i<strlen(header)+1 ; i++ ) _buffer[i] = ' ';
		_buffer += strlen(header)+1;
		_size -= strlen(header)+1;

		sprintf( _buffer , "%s\n" , functionName );
		_buffer += strlen(functionName)+1;
		_size -= strlen(functionName)+1;

		// Line 3
		for( int i=0 ; i<strlen(header)+1 ; i++ ) _buffer[i] = ' ';
		_buffer += strlen(header)+1;
		_size -= strlen(header)+1;

		vsnprintf( _buffer , _size+1 , format , args );

		return buffer;
	}

	struct Exception : public std::exception
	{
		const char *what( void ) const noexcept { return _message.c_str(); }
		template< typename ... Args >
		Exception( const char *fileName , int line , const char *functionName , const char *format , Args ... args )
		{
			char *buffer = MakeMessageString( "[EXCEPTION]" , fileName , line , functionName , format , args ... );
			_message = std::string( buffer );
			delete[] buffer;
		}
	private:
		std::string _message;
	};

	template< typename ... Args > void Throw( const char *fileName , int line , const char *functionName , const char *format , Args ... args ){ throw Exception( fileName , line , functionName , format , args ... ); }
	template< typename ... Args >
	void Warn( const char *fileName , int line , const char *functionName , const char *format , Args ... args )
	{
		char *buffer = MakeMessageString( "[WARNING]" , fileName , line , functionName , format , args ... );
		fprintf( stderr , "%s\n" , buffer );
		delete[] buffer;
	}
	template< typename ... Args >
	void ErrorOut( const char *fileName , int line , const char *functionName , const char *format , Args ... args )
	{
		char *buffer = MakeMessageString( "[ERROR]" , fileName , line , functionName , format , args ... );
		fprintf( stderr , "%s\n" , buffer );
		delete[] buffer;
		exit( 0 );
	}
#else // !VERBOSE_MESSAGING
	inline char *MakeMessageString( const char *header , const char *functionName , const char *format , ... )
	{
		va_list args;
		va_start( args , format );

		size_t _size , size = vsnprintf( NULL , 0 , format , args );
		size += strlen(header)+1;
		size += strlen(functionName)+2;

		char *_buffer , *buffer = new char[ size+1 ];
		_size = size , _buffer = buffer;

		sprintf( _buffer , "%s " , header );
		_buffer += strlen(header)+1;
		_size -= strlen(header)+1;

		sprintf( _buffer , "%s: " , functionName );
		_buffer += strlen(functionName)+2;
		_size -= strlen(functionName)+2;

		vsnprintf( _buffer , _size+1 , format , args );

		return buffer;
	}
	struct Exception : public std::exception
	{
		const char *what( void ) const noexcept { return _message.c_str(); }
		template< typename ... Args >
		Exception( const char *functionName , const char *format , Args ... args )
		{
			char *buffer = MakeMessageString( "[EXCEPTION]" , functionName , format , args ... );
			_message = std::string( buffer );
			delete[] buffer;
		}
	private:
		std::string _message;
	};
	template< typename ... Args > void Throw( const char *functionName , const char *format , Args ... args ){ throw Exception( functionName , format , args ... ); }
	template< typename ... Args >
	void Warn( const char *functionName , const char *format , Args ... args )
	{
		char *buffer = MakeMessageString( "[WARNING]" , functionName , format , args ... );
		fprintf( stderr , "%s\n" , buffer );
		delete[] buffer;
	}
	template< typename ... Args >
	void ErrorOut( const char *functionName , const char *format , Args ... args )
	{
		char *buffer = MakeMessageString( "[ERROR]" , functionName , format , args ... );
		fprintf( stderr , "%s\n" , buffer );
		delete[] buffer;
		exit( 0 );
	}
#endif // VERBOSE_MESSAGING
}
#ifdef VERBOSE_MESSAGING
#ifndef WARN
#define WARN( ... ) Util::Warn( __FILE__ , __LINE__ , __FUNCTION__ , __VA_ARGS__ )
#endif // WARN
#ifndef WARN_ONCE
#define WARN_ONCE( ... ) { static bool firstTime = true ; if( firstTime ) Util::Warn( __FILE__ , __LINE__ , __FUNCTION__ , __VA_ARGS__ ) ; firstTime = false; }
#endif // WARN_ONCE
#ifndef THROW
#define THROW( ... ) Util::Throw( __FILE__ , __LINE__ , __FUNCTION__ , __VA_ARGS__ )
#endif // THROW
#ifndef ERROR_OUT
#define ERROR_OUT( ... ) Util::ErrorOut( __FILE__ , __LINE__ , __FUNCTION__ , __VA_ARGS__ )
#endif // ERROR_OUT
#else // !VERBOSE_MESSAGING
#ifndef WARN
#define WARN( ... ) Util::Warn( __FUNCTION__ , __VA_ARGS__ )
#endif // WARN
#ifndef WARN_ONCE
#define WARN_ONCE( ... ) { static bool firstTime = true ; if( firstTime ) Util::Warn( __FUNCTION__ , __VA_ARGS__ ) ; firstTime = false; }
#endif // WARN_ONCE
#ifndef THROW
#define THROW( ... ) Util::Throw( __FUNCTION__ , __VA_ARGS__ )
#endif // THROW
#ifndef ERROR_OUT
#define ERROR_OUT( ... ) Util::ErrorOut( __FUNCTION__ , __VA_ARGS__ )
#endif // ERROR_OUT
#endif // VERBOSE_MESSAGING
#endif // EXCEPTIONS_INCLUDED
