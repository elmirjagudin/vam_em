// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the VAM_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// VAM_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef VAM_EXPORTS
#define VAM_API extern "C" __declspec(dllexport)
#else
#define VAM_API extern "C" __declspec(dllimport)
#endif

typedef struct vam_handle_s vam_handle;

VAM_API vam_handle*
vam_init(int width, int height);

VAM_API void
vam_process_frame(vam_handle *vah, void *pixels);
