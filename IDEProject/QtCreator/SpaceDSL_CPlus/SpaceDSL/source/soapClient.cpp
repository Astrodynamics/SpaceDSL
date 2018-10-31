/* soapClient.cpp
   Generated by gSOAP 2.8.66 for IERS.h

gSOAP XML Web services tools
Copyright (C) 2000-2018, Robert van Engelen, Genivia Inc. All Rights Reserved.
The soapcpp2 tool and its generated software are released under the GPL.
This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#if defined(__BORLANDC__)
#pragma option push -w-8060
#pragma option push -w-8004
#endif
#include "../Include/gsoap/soapH.h"

SOAP_SOURCE_STAMP("@(#) soapClient.cpp ver 2.8.66 2018-04-20 02:12:53 GMT")


SOAP_FMAC5 int SOAP_FMAC6 soap_call_ns1__readEOP(struct soap *soap, const char *soap_endpoint, const char *soap_action, char *param, char *series, char *mjd, char *&return_)
{	struct ns1__readEOP soap_tmp_ns1__readEOP;
	struct ns1__readEOPResponse *soap_tmp_ns1__readEOPResponse;
	if (soap_endpoint == nullptr)
		soap_endpoint = "https://data.iers.org/eris/webservice/eop/eopServer.php";
	if (soap_action == nullptr)
		soap_action = "urn:org.iers.data.eop#readEOP";
	soap_tmp_ns1__readEOP.param = param;
	soap_tmp_ns1__readEOP.series = series;
	soap_tmp_ns1__readEOP.mjd = mjd;
	soap_begin(soap);
	soap->encodingStyle = "http://schemas.xmlsoap.org/soap/encoding/";
	soap_serializeheader(soap);
	soap_serialize_ns1__readEOP(soap, &soap_tmp_ns1__readEOP);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put_ns1__readEOP(soap, &soap_tmp_ns1__readEOP, "ns1:readEOP", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put_ns1__readEOP(soap, &soap_tmp_ns1__readEOP, "ns1:readEOP", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	return_ = nullptr;
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	if (soap_recv_fault(soap, 1))
		return soap->error;
	soap_tmp_ns1__readEOPResponse = soap_get_ns1__readEOPResponse(soap, nullptr, "", nullptr);
	if (!soap_tmp_ns1__readEOPResponse || soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return_ = soap_tmp_ns1__readEOPResponse->return_;
	return soap_closesock(soap);
}

SOAP_FMAC5 int SOAP_FMAC6 soap_call_ns2__getTimescale(struct soap *soap, const char *soap_endpoint, const char *soap_action, char *param, char *datetime, char *&return_)
{	struct ns2__getTimescale soap_tmp_ns2__getTimescale;
	struct ns2__getTimescaleResponse *soap_tmp_ns2__getTimescaleResponse;
	if (soap_endpoint == nullptr)
		soap_endpoint = "https://data.iers.org/eris/webservice/timescales/timescalesServer.php";
	if (soap_action == nullptr)
		soap_action = "urn:org.iers.data.timescales#getTimescale";
	soap_tmp_ns2__getTimescale.param = param;
	soap_tmp_ns2__getTimescale.datetime = datetime;
	soap_begin(soap);
	soap->encodingStyle = "http://schemas.xmlsoap.org/soap/encoding/";
	soap_serializeheader(soap);
	soap_serialize_ns2__getTimescale(soap, &soap_tmp_ns2__getTimescale);
	if (soap_begin_count(soap))
		return soap->error;
	if (soap->mode & SOAP_IO_LENGTH)
	{	if (soap_envelope_begin_out(soap)
		 || soap_putheader(soap)
		 || soap_body_begin_out(soap)
		 || soap_put_ns2__getTimescale(soap, &soap_tmp_ns2__getTimescale, "ns2:getTimescale", "")
		 || soap_body_end_out(soap)
		 || soap_envelope_end_out(soap))
			 return soap->error;
	}
	if (soap_end_count(soap))
		return soap->error;
	if (soap_connect(soap, soap_endpoint, soap_action)
	 || soap_envelope_begin_out(soap)
	 || soap_putheader(soap)
	 || soap_body_begin_out(soap)
	 || soap_put_ns2__getTimescale(soap, &soap_tmp_ns2__getTimescale, "ns2:getTimescale", "")
	 || soap_body_end_out(soap)
	 || soap_envelope_end_out(soap)
	 || soap_end_send(soap))
		return soap_closesock(soap);
	return_ = nullptr;
	if (soap_begin_recv(soap)
	 || soap_envelope_begin_in(soap)
	 || soap_recv_header(soap)
	 || soap_body_begin_in(soap))
		return soap_closesock(soap);
	if (soap_recv_fault(soap, 1))
		return soap->error;
	soap_tmp_ns2__getTimescaleResponse = soap_get_ns2__getTimescaleResponse(soap, nullptr, "", nullptr);
	if (!soap_tmp_ns2__getTimescaleResponse || soap->error)
		return soap_recv_fault(soap, 0);
	if (soap_body_end_in(soap)
	 || soap_envelope_end_in(soap)
	 || soap_end_recv(soap))
		return soap_closesock(soap);
	return_ = soap_tmp_ns2__getTimescaleResponse->return_;
	return soap_closesock(soap);
}

#if defined(__BORLANDC__)
#pragma option pop
#pragma option pop
#endif

/* End of soapClient.cpp */
