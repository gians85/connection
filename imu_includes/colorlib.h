/*
 * File:   colorlib.h
 * Author: Gianluca
 * Comments: terminal output color. Thanks to epatel@stackoverflow
 *           and http://wiki.bash-hackers.org/scripting/terminalcodes
 * Revision history: 0.1
 *
 * example:
 *      printf( ESC GREEN "Here is some text\n" ESC RESET );
 */

#ifndef IMU_COLORLIB_H_
#define IMU_COLORLIB_H_


#define ESC     "\033"
#define RST_ST  "\033[0m"
#define BLACK   "\033[30m"      // Black
#define RED     "\033[31m"      // Red
#define GREEN   "\033[32m"      // Green
#define YELLOW  "\033[33m"      // Yellow
#define BLUE    "\033[34m"      // Blue
#define MAGENTA "\033[35m"      // Magenta
#define CYAN    "\033[36m"      // Cyan
#define WHITE   "\033[37m"      // White

#define BOLD    "\033[1m"

#define BLACK_BG   "\033[40m"      // Background Black
#define RED_BG     "\033[41m"      // Background Red
#define GREEN_BG   "\033[42m"      // Background Green
#define YELLOW_BG  "\033[43m"      // Background Yellow
#define BLUE_BG    "\033[44m"      // Background Blue
#define MAGENTA_BG "\033[45m"      // Background Magenta
#define CYAN_BG    "\033[46m"      // Background Cyan
#define WHITE_BG   "\033[47m"      // Background White

#define CLEARLINE  "\033[0K"    //Clear line from current cursor position to end of line
#define CLEARLINE1 "\033[1K"    //Clear line from beginning to current cursor position
#define CLEARLINE2 "\033[2K"	//Clear whole line (cursor position unchanged)

#define BEL "\a"    // Terminal bell
#define BS	"\b"	// Backspace
#define VT	"\013"	// Vertical TAB \v
#define DEL	"\177"  // Delete character
#define CR  "\r"    // Carriage Return
#define NL  "\n"    // New Line

#define CLS	"\014"	// Formfeed (also: New page NP)

#define CURSOR_ON "\033[?25h"
#define CURSOR_OFF  "\033[?25l"



#endif /* IMU_COLORLIB_H_ */
