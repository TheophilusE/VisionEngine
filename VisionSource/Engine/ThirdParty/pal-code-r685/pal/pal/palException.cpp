/*
 * palException.cpp
 *
 *  Created on: Jul 27, 2010
 *      Author: Chris Long
 *  Copyright (C) 2010 SET Corporation
 */

#include <pal/palException.h>
#include <string.h>
#include <malloc.h>
#include <stdlib.h>

#ifdef _MSC_VER
#define strdup _strdup
#endif

palException::palException() throw()
    : m_message(0), m_cause(0)
{
}

palException::palException(const palException& other) throw()
    : m_message(strdup(other.what())),
      m_cause(other.GetCause())
{
}

palException::palException(const char* message, const std::exception *cause) throw()
    : m_message(strdup(message ? message : 0)), m_cause(cause)
{
}

palException::~palException() throw() {
    if (m_message) {
        free((void*) m_message);
        m_message = 0;
    }
    delete m_cause;
    m_cause = 0;
}

const std::exception *palException::GetCause() const {
    return m_cause;
}

const char *palException::what() const throw() {
    return m_message == 0 ? "" : m_message;
}

void palException::InitMessage(const char *message) {
    if (!m_message && message != NULL) {
        m_message = strdup(message);
    }
}

void palException::InitCause(std::exception *cause) {
    if (!m_cause) {
        m_cause = cause;
    };
}
