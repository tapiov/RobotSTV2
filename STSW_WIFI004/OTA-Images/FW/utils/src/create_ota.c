#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <openssl/evp.h>
#include <openssl/rand.h>

#include "ota_update_hdr.h"
#include "version.h"
#include "common.h"



#define G_BUF_LEN (1024*10)

extern filetype    Filename;           /* string for opening files */

int hex2bin_main(int argc, char **argv);

int main(int argc, char **argv) {
	struct ota_update_hdr hdr; /*Header struct */
	FILE *f_in=NULL, *f_out=NULL, *f_key=NULL; /* Files */
	uint8_t buf[G_BUF_LEN]; /* Temporary Buffer */
	const uint32_t hexspeak = G_FINAL_HEXSPEAK; /* Final Hexspeak */
  char system_version[MAX_STR_LEN];   /* Version */
	int32_t i=0, retval=0; /* Supporing ints */
	struct stat sbuf; /* to store file size */
  uint8_t key[16], iv[12]={0,}, tag[16]={0,}; /* key iv and tag for AES-128-GCM */
  int32_t encrypt=0;     /* flag specifying if encryption is to be used  */  
  EVP_CIPHER_CTX *ctx = NULL;  /* OpenSSL context for AES-128-GCM */
  EVP_MD_CTX *mdctx = NULL; /* OpenSSL context for SHA-256 */
  uint8_t cipher[G_BUF_LEN]; /* Temporary Buffer to store encrypted buf*/
  int outlen;
  
  OpenSSL_add_all_digests();

	if (argc < 3) 
  {
		fprintf(stderr, "\nusage: %s infile outfile [keyfile]\n\nIf keyfile is omitted, the resulting ota file will not be encrypted and authenticated.\n\n",argv[0]);
		exit(1);
	}

  retval = hex2bin_main(2,argv);
  if (retval != 0)
  {
    return(retval);
  }
  
  
  
	f_in = fopen(Filename, "r");
	f_out = fopen(argv[2], "w");

	if (!f_in || !f_out)
  {
    fprintf(stderr, "ERROR: Input file or output file could not be opened\n");
    unlink(Filename);
		exit(2);
  }
  /* Set header to zero */
  memset(&hdr, 0, sizeof(hdr));
  
  /* Do we have a key file ? */
  if (argc == 4)
  {
    /* Yes, then open it and read the key. */
    f_key = fopen(argv[3], "rb");
    if (f_key == NULL)
    {
      fprintf(stderr, "ERROR: Key file could not be opened\n");
      unlink(Filename);
      exit(2);
    }        
    i = fread(key, 1, 16, f_key);
    if (i != 16)
    {
      fprintf(stderr, "ERROR: Key file is smaller than the required 128 bits\n");
      unlink(Filename);
      exit(-3);
    }    
    encrypt = 1;    
    /* Generate random iv, check it's not made by all zeros */
    do 
    {      
      retval = RAND_bytes(iv, sizeof(iv));
      if (retval != 1 )
      { 
        fprintf(stderr, "ERROR: Unexpected error in random number generation.\n");
        unlink(Filename);
        exit(-4);
      }
    } while (memcmp(iv, (uint8_t *) &hdr, 12)==0);
  }
  
  /* Read FW size */
	stat(Filename, &sbuf);
  /* Which should be a multiple of 4*/  
  if (sbuf.st_size % 4 != 0)
  {
		fprintf(stderr, "ERROR:  Input file size is %d which is not multiple of 4 bytes. This is unsupported.\n", (int32_t) sbuf.st_size);
    unlink(Filename);
		exit(-128);    
  }
	 
  /* Create SYSTEM_VERSION as "SYSTEM_DATE-SYSTEM_BUILD" */
  strncat(system_version, SYSTEM_DATE, sizeof(system_version));
  strncat(system_version, "-", sizeof(system_version) - strlen(system_version));
  strncat(system_version, SYSTEM_BUILD, sizeof(system_version) - strlen(system_version) - 1 );

	if (strlen(system_version) >= sizeof(hdr.version)) 
  {
		/* if C99 compiler, use %zu,
		 * else, 			use %lu and (unsigned long) cast */
		fprintf(stderr, "ERROR:  VERSION string too long! (%zu/%zu max)\n", strlen(system_version), sizeof(hdr.version)-1);
    unlink(Filename);
		exit(-128);
	}  
  
  /* Set ID */  
	hdr.id = G_UPDATE_COOKIE;
	/* Set HDR size  */
  hdr.hdrlen = sizeof(hdr);
  /* Set Base */
	hdr.base = G_BASE;
	/* Set FW size  */
  hdr.len = sbuf.st_size;
  /* Copy Model */
	strncpy((char*)hdr.model, SYSTEM_MODEL, sizeof(hdr.model));	
  /* Copy version */
	strncpy((char*)hdr.version, system_version, sizeof(hdr.version));

  /* Pre hash the file */
  mdctx = EVP_MD_CTX_create();
  retval = EVP_DigestInit_ex(mdctx, EVP_sha256(), NULL);  
  if (retval != 1)
  {
    fprintf(stderr, "ERROR:  Error in initializing SHA-256 Context!\n");
    unlink(Filename);
		exit(-128);
  }  
  while((i = fread(buf, 4, G_BUF_LEN/4, f_in)) > 0) 
  {   
    EVP_DigestUpdate(mdctx, buf, i*4);    
	}  
  retval = EVP_DigestFinal_ex(mdctx, hdr.hash, NULL);  
    
  if (retval != 1 )
  {
    fprintf(stderr, "ERROR:  Error in generating SHA-256 Digest!\n");
    unlink(Filename);
		exit(-128);
  }
  EVP_MD_CTX_destroy(mdctx);
    
  /* If required, pre process in AES-GCM-128 to get the TAG */
  if (encrypt == 1)
  {
    /* Roll back to start of f_in */
    fseek ( f_in , 0 , SEEK_SET );
    /* Create Context */
    if(!(ctx = EVP_CIPHER_CTX_new())) 
    {
      fprintf(stderr, "ERROR:  Error in creating AES-GCM Context!\n");
      unlink(Filename);
      exit(-32);
    }
    /* Initialise the encryption operation. */
    if(1 != EVP_EncryptInit_ex(ctx, EVP_aes_128_gcm(), NULL, key, iv))
    {
      fprintf(stderr, "ERROR:  Error in initializing AES-GCM Context!\n");
      unlink(Filename);
      exit(-33);
    }
    /* Processing Header */
    if(1 != EVP_EncryptUpdate(ctx, NULL, &outlen, (uint8_t *) &hdr, sizeof(hdr)))
    {
      fprintf(stderr, "ERROR:  Error in processing header with AES-GCM\n");
      unlink(Filename);
      exit(-34);
    }
    while((i = fread(buf, 4, G_BUF_LEN/4, f_in)) > 0) 
    {   
      EVP_EncryptUpdate(ctx, cipher, &outlen, buf, i*4);
    } 
    retval = EVP_EncryptFinal_ex(ctx, cipher, &outlen);
    if(retval != 1 || outlen != 0)
    {
      fprintf(stderr, "ERROR:  Error in finalizing encryption with AES-GCM\n");
      unlink(Filename);
      exit(-35);     
    }    
    if(1 != EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_GET_TAG, 16, tag))
    {
      fprintf(stderr, "ERROR:  Error in obtaining the tag with AES-GCM\n");
      unlink(Filename);
      exit(-36);     
    }          
    EVP_CIPHER_CTX_free(ctx);
  }
  
  /* File is hashed and the result is in hdr.hash */  
  
  
  /* Roll back to start of f_in */
  fseek ( f_in , 0 , SEEK_SET );
  /* Copy IV in the strcuture */
  memcpy(hdr.iv,iv,12);
  memcpy(hdr.tag,tag,16);
  /* Start writing header to f_out*/
	fwrite(&hdr, 1, sizeof(hdr), f_out);    
  /* Reset to zero the fields that are not authenticated by GCM */
  memset(hdr.iv,0x00,12);
  memset(hdr.tag,0x00,16);
  /* Re-read file and write it. But compute hash to check */  
  mdctx = EVP_MD_CTX_create();
  EVP_DigestInit_ex(mdctx, EVP_sha256(), NULL);    
  if (retval != 1)
  {
    fprintf(stderr, "ERROR:  Error in initializing SHA-256 Context!\n");
    unlink(Filename);
		exit(-128);
  }  
  
  /* If required, process in AES-GCM-128 to get the ciphertext */
  if (encrypt == 1)
  {
    /* Create Context */
    if(!(ctx = EVP_CIPHER_CTX_new())) 
    {
      fprintf(stderr, "ERROR:  Error in creating AES-GCM Context!\n");
      unlink(Filename);
      exit(-32);
    }
    /* Initialise the encryption operation. */
    if(1 != EVP_EncryptInit_ex(ctx, EVP_aes_128_gcm(), NULL, key, iv))
    {
      fprintf(stderr, "ERROR:  Error in initializing AES-GCM Context!\n");
      unlink(Filename);
      exit(-33);
    }
    /* Processing Header */
    if(1 != EVP_EncryptUpdate(ctx, NULL, &outlen, (uint8_t *) &hdr, sizeof(hdr)))
    {
      fprintf(stderr, "ERROR:  Error in processing header with AES-GCM\n");
      unlink(Filename);
      exit(-34);
    }
  }
  
	while((i = fread(buf, 4, G_BUF_LEN/4, f_in)) > 0) 
  {    
    EVP_DigestUpdate(mdctx, buf, i*4); /* Update HASH*/
    if (encrypt == 1)
    {
      EVP_EncryptUpdate(ctx, cipher, &outlen, buf, i*4);
      fwrite(cipher, 4, i, f_out);     /* Write out  encrypted data*/
    }
    else
    {
      fwrite(buf, 4, i, f_out);     /* Write out plaintext data */
    }
	}
  /* Retrieve digest and compare */
  retval = EVP_DigestFinal_ex(mdctx, buf, NULL);  
  if (retval != 1  || memcmp(buf,hdr.hash,32) !=0 )
  {
    fprintf(stderr, "ERROR:  Error in checking the SHA-256 Digest!\n");
    unlink(Filename);
		exit(-128);
  }
  
  if (encrypt == 1)
  {
    /* Generate TAG and compare */
    retval = EVP_EncryptFinal_ex(ctx, cipher, &outlen);
    if(retval != 1 || outlen != 0)
    {
      fprintf(stderr, "ERROR:  Error in finalizing encryption with AES-GCM\n");
      unlink(Filename);
      exit(-35);     
    } 
    retval = EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_GET_TAG, 16, buf);
    if(1 != retval || memcmp(buf,tag,16) != 0 )
    {
      fprintf(stderr, "ERROR:  Error in checking the tag with AES-GCM\n");
      unlink(Filename);
      exit(-37);     
    }
    if (ctx != NULL)
    {
      EVP_CIPHER_CTX_free(ctx);
    }
  }    
  
  /* Write hexspeak */
	fwrite(&hexspeak, 4, 1, f_out);

	fclose(f_in);
	fclose(f_out);
	EVP_MD_CTX_destroy(mdctx);
  EVP_cleanup();

  printf("Operation Completed Successfully\n");
  unlink(Filename);
	return 0;
}
