%include "macros.inc"

%ifdef WIN32
%define kalmanFilter_Iterate _kalmanFilter_Iterate
%endif

%define	angle 	 	dword[ebp - 4]		; *angle  	 @ ebp - 8
%define	rateBias 	dword[ebp - 8]		; *rateBias  @ ebp - 8
%define	angleRate	dword[ebp - 12]		; *angleRate @ ebp - 12
%define	dt			dword[ebp - 16]		; *dt		 @ ebp - 16

%define	P[0][0]		dword[ebp - 20]		; P[0][0]	 @ ebp - 20
%define	P[0][1]		dword[ebp - 24]		; P[0][1]	 @ ebp - 24
%define	P[1][0]		dword[ebp - 28]		; P[1][0]	 @ ebp - 28
%define	P[1][1]		dword[ebp - 32]		; P[1][1]	 @ ebp - 32

%define	K[0]		dword[ebp - 36]		; K[0]	 	 @ ebp - 36
%define	K[1]		dword[ebp - 40]		; K[1]		 @ ebp - 40

%define	innovation	dword[ebp - 44]		; *innovation@ ebp - 44
%define	S			dword[ebp - 48]		; *S@		 @ ebp - 48
%define	Q_rateBias	dword[ebp - 52]		; *Q_rateBias@ ebp - 52
%define	Q_angle		dword[ebp - 56]		; *Q_angle	 @ ebp - 56
%define	u_angle		dword[ebp - 60]		; *u_angle	 @ ebp - 60

section .data
    string  db  "ASM Result: %d",10,0

;extern  _printf

globalfunc kalmanFilter_Iterate, filter:dword, measuredRate:dword, measuredScalar:dword
	
	prologue
	mov		eax, .measuredRate	; ADDRESS !!!!!!
	mov		ebx, .filter		; ADDRESS !!!!!!
	
	; Pushing filter variables into stack - Temporary variables
	push	dword[ebx]			; *angle 	 @ ebp - 4
	push	dword[ebx + 4]		; *rateBias  @ ebp - 8
	push	dword[ebx + 8]		; *angleRate @ ebp - 12
	push	dword[ebx + 12]		; *dt		 @ ebp - 16
	
	push	dword[ebx + 16]		; P[0][0]	 @ ebp - 20
	push	dword[ebx + 20]		; P[0][1]	 @ ebp - 24
	push	dword[ebx + 24]		; P[1][0]	 @ ebp - 28
	push	dword[ebx + 28]		; P[1][1]	 @ ebp - 32
	
	push	dword[ebx + 32]		; K[0]	 	 @ ebp - 36
	push	dword[ebx + 36]		; K[1]		 @ ebp - 40
	
	push	dword[ebx + 40]		; *innovation@ ebp - 44
	push	dword[ebx + 44]		; *S@		 @ ebp - 48
	push	dword[ebx + 48]		; *Q_rateBias@ ebp - 52
	push	dword[ebx + 52]		; *Q_angle	 @ ebp - 56
	push	dword[ebx + 56]		; *u_angle	 @ ebp - 60
	
	
	; // Prediction op1: angleRate = measuredRate - rateBias;
	fld		dword [eax]			; st0 = measuredRate
	fsub	rateBias			; st0 -= rateBias
	fst     angleRate			; store st0 @ filter->angleRate
	
	; // Prediction op2: angle += angleRate * dt;
	fmul	dt					; st0 *= dt
	fadd	angle				; st0 += angle
	fst    	angle				; store st0 @ angle 
	; -----1 element in st----- st0 = angle
	
	; // Innovation: innovation = measuredScalar - angle;
	mov		eax, .measuredScalar
	fchs						; Change the sign of st0 //angle
	fadd	dword [eax]			; measuredScalar - angle;
	fstp    innovation			; store and pop st0 @ innovation
	; -----0 elements in st-----
	
	; // Prediction covariance matrix op1: 
	; P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
	fld		dt					; st0 = dt
	fld		dt					; st0 = dt, st1 = dt
	fmul	P[1][1]				; st0 *= P[1][1]
	fsub	P[0][1]				; st0 -= P[0][1]
	fsub	P[1][0]				; st0 -= P[1][0]
	fadd	Q_angle				; st0 += Q_angle
	fmul 	st(0),st(1)			; st0 *= st1 //dt
	fst		P[0][0]				; P[0][0] = st0
	; -----2 elements in st----- st0 = P[0][0], st1 = dt
	
	; // Innovation error
	; S = P[0][0] + u_angle;
	fld		u_angle				; st0 = u_angle, st1 = P[0][0], st2 = dt
	fadd	st(0),st(1)			; st0 += P[0][0]
	fst		S					; S = st0
	
	; P[0][1] -= filter->dt * P[1][1];
	; P[1][0] -= filter->dt * P[1][1];
	; P[1][1] += filter->Q_rateBias * filter->dt;
	
	
	
	mov		eax,	angle
	mov		[ebx], eax

	xor		eax, eax
	
	; push	__float32__( 2.0 )
	; fld		dword [esp]	
	; add		esp,	4
	
;---------------------------------------
    ; // Prediction
	;+ *angleRateTemp = measuredRate - *rateBiasTemp;
	;+ *angleTemp += *angleRateTemp * filter->dt;

	; // Prediction covariance matrix
	;+ P[0][0] += filter->dt * (filter->dt*P[1][1] - P[0][1] - P[1][0] + filter->Q_angle);
	; P[0][1] -= filter->dt * P[1][1];
	; P[1][0] -= filter->dt * P[1][1];
	; P[1][1] += filter->Q_rateBias * filter->dt;

	; // Innovation
	;+filter->innovation = measuredScalar - *angleTemp;

	; // Innovation error
	;- filter->S = P[0][0] + filter->u_angle;

	; // Kalman gain
	;- K[0] = P[0][0] / filter->S;
	; K[1] = P[1][0] / filter->S;

	; // Posteriori angle
	; *angleTemp += K[0] * filter->innovation;
	; *rateBiasTemp += K[1] * filter->innovation;

	; // Posteriori covariance matrix
	;- P[0][0] -= K[0] * P[0][0];
	; P[0][1] -= K[0] * P[0][1];
	; P[1][0] -= K[1] * P[0][0];
	; P[1][1] -= K[1] * P[0][1];
;}

	epilogue

%ifdef WIN32
	extern _ExitProcess@4
	globalfunc _aexit, exitCode:dword
		prologue
		invoke _ExitProcess@4, .exitCode
		epilogue
%else
	globalfunc aexit, exitCode:dword
		uses ebx
		prologue
		mov eax, 0x01      ; SYS_EXIT (terminate process)
		mov ebx, .exitCode ; exitcode
		int 0x80           ; syscall interrupt
		epilogue
%endif	