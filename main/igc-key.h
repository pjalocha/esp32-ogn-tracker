#ifndef __IGC_KEY_H__
#define __IGC_KEY_H__

// #define MBEDTLS_ECDSA_DETERMINISTIC

// #include "mbedtls/md5.h"
#include "mbedtls/sha256.h"
#include "mbedtls/rsa.h"
#include "mbedtls/platform.h"
#include "mbedtls/x509_csr.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/sha256.h"
#include "mbedtls/ecp.h"
#include "mbedtls/pk.h"
#include "mbedtls/version.h"

class SHA256
{ public:
   mbedtls_sha256_context Context;

  public:
   void Init(void)                               { mbedtls_sha256_init(&Context); }
   void Free(void)                               { mbedtls_sha256_free(&Context); }
   int  Start(void)                              { return mbedtls_sha256_starts_ret(&Context, 0); }
   int  Update(const uint8_t *Input, size_t Len) { return mbedtls_sha256_update_ret(&Context, Input, Len); }
   void Clone(const SHA256 &Src)                 { mbedtls_sha256_clone(&Context, &Src.Context); }
   int  Finish(uint8_t CheckSum[32])             { return mbedtls_sha256_finish_ret(&Context, CheckSum); }

} ;

class SHA512
{ public:
   mbedtls_sha512_context Context;

  public:
   void Init(void)                              { mbedtls_sha512_init(&Context); }
   void Free(void)                              { mbedtls_sha512_free(&Context); }
   int Start(void)                              { return mbedtls_sha512_starts_ret(&Context, 0); }
   int Update(const uint8_t *Input, size_t Len) { return mbedtls_sha512_update_ret(&Context, Input, Len); }
   int Finish(uint8_t CheckSum[64])             { return mbedtls_sha512_finish_ret(&Context, CheckSum); }

} ;

// Uncomment to force use of a specific curve
#define ECPARAMS    MBEDTLS_ECP_DP_SECP256K1

#if !defined(ECPARAMS)
#define ECPARAMS    mbedtls_ecp_curve_list()->grp_id
#endif

class IGC_Key
{ public:
   mbedtls_ecdsa_context SignCtx; // this seems to be the key-pair as it is defined: typedef mbedtls_ecp_keypair mbedtls_ecdsa_context;

   mbedtls_ctr_drbg_context CtrDrbgCtx; // RNG parameter, used to produce the key but as well to produce signature

   mbedtls_pk_context Key;
   mbedtls_x509write_csr Req;
   mbedtls_entropy_context Entropy;

   static const uint8_t PrivBinSize = 36; // 72;                      // [bytes] max. size for the private key in binary form

 public:
   // IGC_Key() { Init(); }

   int Init(void)                                              // initialize on startup
   { const char *Pers = "ecdsa";
     mbedtls_x509write_csr_init(&Req);
     mbedtls_pk_init(&Key);
     mbedtls_ecdsa_init(&SignCtx);
     mbedtls_ctr_drbg_init(&CtrDrbgCtx);
     mbedtls_entropy_init(&Entropy);
     int Ret = mbedtls_ctr_drbg_seed( &CtrDrbgCtx, mbedtls_entropy_func, &Entropy,
                                      (const unsigned char *)Pers, strlen(Pers) );
     return Ret; }                                             // return zero on success

   int Generate(void)                                          // produce a new pair of keys: private and public key
   {                                                           // key-pair, curve, RNG function, RNG parameter
     int Ret = mbedtls_ecdsa_genkey(&SignCtx, ECPARAMS, mbedtls_ctr_drbg_random, &CtrDrbgCtx);  // produce key-pair
     if(Ret!=0) return Ret;
     Ret = mbedtls_pk_setup(&Key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
     if(Ret!=0) return Ret;
     Key.pk_ctx = &SignCtx;                                    // ?
     return Ret; }                                             // return zero on success

   int Write(uint8_t *Data, int MaxLen=240)                     // write both private and public keys to a binary record
   { if(MaxLen<=PrivBinSize) return 0;
     if(Priv_WriteBin(Data, PrivBinSize)!=0) return 0;
     int Len=Pub_WriteBin(Data+PrivBinSize, MaxLen-PrivBinSize); if(Len==0) return 0;
     return Len+PrivBinSize; }                                 // return the number of bytes

   int Read(const uint8_t *Data, int Len)
   { if(Len<=PrivBinSize) return 0;
     if(Priv_ReadBin(Data, PrivBinSize)!=0) return 0;
     if(Pub_ReadBin(Data+PrivBinSize, Len-PrivBinSize)!=0) return 0;
     // int Ret = mbedtls_pk_setup(&Key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
     Key.pk_ctx = &SignCtx;                                    // ?
     return Len; }                                             // return number of bytes read

   int Sign_MD5_SHA256(uint8_t *Sign, const uint8_t *Hash, int HashLen)      // sign an MD5/SHA256 Hash
   { size_t SignLen=0;
     int Ret=mbedtls_ecdsa_write_signature(&SignCtx, MBEDTLS_MD_SHA256, Hash, HashLen, Sign, &SignLen, mbedtls_ctr_drbg_random, &CtrDrbgCtx);
     if(Ret!=0) return 0;                                      // return zero if failure
     return SignLen; }                                         // return the size of the signature

   int Sign_MD5_SHA512(uint8_t *Sign, const uint8_t *Hash, int HashLen)      // sign an MD5/SHA512 Hash
   { size_t SignLen=0;
     int Ret=mbedtls_ecdsa_write_signature(&SignCtx, MBEDTLS_MD_SHA512, Hash, HashLen, Sign, &SignLen, mbedtls_ctr_drbg_random, &CtrDrbgCtx);
     if(Ret!=0) return 0;                                      // return zero if failure
     return SignLen; }                                         // return the size of the signature

   int Verify_SHA256(unsigned char *Sign, int sig_len, const uint8_t *Hash, int HashLen)      // Verify a signature
   { int Ret=mbedtls_ecdsa_read_signature (&SignCtx, Hash, HashLen, Sign, sig_len);
     return Ret; }                                         // return the size of the signature

   int Pub_WriteBin(uint8_t *Data, int MaxLen)                 // write the public key in a binary form
   { size_t Len=0;
     if(mbedtls_ecp_point_write_binary(&SignCtx.grp, &SignCtx.Q, MBEDTLS_ECP_PF_UNCOMPRESSED, &Len, Data, MaxLen)!=0) return 0;
     return Len; }                                             // return number of bytes written

   int Pub_ReadBin(const uint8_t *Data, int Len)
   { return mbedtls_ecp_point_read_binary(&SignCtx.grp, &SignCtx.Q, Data, Len); } // return zero for success

   int Priv_WriteBin(uint8_t *Data, int Len=PrivBinSize)       // write the private key in a binary form
   { return mbedtls_mpi_write_binary(&SignCtx.d, Data, Len); } // return zero if success (always fills the whole buffer adding leading zeros)

   int Priv_ReadBin(const uint8_t *Data, int Len=PrivBinSize)  // read the private key in the binary form
   { return mbedtls_mpi_read_binary(&SignCtx.d, Data, Len); }  // return zero for success

   int Pub_Write(uint8_t *Out, int MaxLen)                     // write the public key in an ASCII form
   { return mbedtls_pk_write_pubkey_pem(&Key, Out, MaxLen); }  // return zero if success

   int Priv_Write(uint8_t *Out, int MaxLen)                    // write the private key in an ASCII form
   { return mbedtls_pk_write_key_pem(&Key, Out, MaxLen); }     // return zero if success

   int Pub_Write_DER(uint8_t *Out, int MaxLen)                     // write the public key in an ASCII form
   { return mbedtls_pk_write_pubkey_der(&Key, Out, MaxLen); }  // return zero if success

   int Priv_Write_DER(uint8_t *Out, int MaxLen)                    // write the private key in an ASCII form
   { return mbedtls_pk_write_key_der(&Key, Out, MaxLen); }     // return zero if success

#ifdef WITH_ESP32
  esp_err_t WriteToNVS(const char *Name="IGCKEY", const char *NameSpace="TRACKER")
  { uint8_t Buff[256];
    nvs_handle Handle;
    esp_err_t Err = nvs_open(NameSpace, NVS_READWRITE, &Handle);
    if(Err!=ESP_OK) return Err;
    int BuffLen = Write(Buff, 256);
    Err = nvs_set_blob(Handle, Name, Buff, BuffLen);
    if(Err==ESP_OK) Err = nvs_commit(Handle);
    nvs_close(Handle);
    return Err; }

  esp_err_t ReadFromNVS(const char *Name="IGCKEY", const char *NameSpace="TRACKER")
  { uint8_t Buff[256];
    nvs_handle Handle;
    esp_err_t Err = nvs_open(NameSpace, NVS_READWRITE, &Handle);
    if(Err!=ESP_OK) return Err;
    size_t Size=0;
    Err = nvs_get_blob(Handle, Name,    0, &Size);                  // get the Size of the blob in the Flash
    if( (Err==ESP_OK) && (Size<=256) )
      Err = nvs_get_blob(Handle, Name, Buff, &Size);                // read the Blob from the Flash
    nvs_close(Handle);
    if(Err!=ESP_OK) return Err;
    if(Read(Buff, Size)==Size) return ESP_OK;
    return ESP_ERR_NOT_FOUND; }
#endif // WITH_ESP32

} ;

#endif // __IGC_KEY_H__
