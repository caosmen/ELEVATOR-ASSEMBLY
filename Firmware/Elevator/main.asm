/* |------------------------------------------------------------------------| */
/* | Implementação Elevador (Firmware)                                      | */
/* |------------------------------------------------------------------------| */
/* | Curso: Engenharia de Computação										| */
/* | Disciplina: Microcontroladores e Aplicações                            | */
/* | Professor: Erick Barboza                                               | */
/* | Alunos: Bruno Lemos, Karla Sophia, Leticia Gabriela e Maria Fernanda   | */
/* |------------------------------------------------------------------------| */

/* ---------------------Definição das variáveis globais---------------------- */
#define CLOCK 16.0e6 
#define DELAY_TIMER_A 1.0

#define PRESCALE_TIMER_A 0b101
#define PRESCALE_DIV_TIMER_A 1024
#define WGM_TIMER_A 0b0100

/* ---------------------Definição das variáveis globais---------------------- */
/* -------------------------------------------------------------------------- */
/* ------------------------Definição dos registradores----------------------- */

// Registrador para uso geral
.def global_temp = R16

// Registradores usados para armazenar o estado do elevador
.def global_state = r20
.def global_current_floor = r21
.def next_floor = r22
.def direction = r23
.def counter = r24

/* ------------------------Definição dos registradores----------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------Definição dos pinos usados----------------------- */

.equ pin_button_external_floor_0 = PB5
.equ pin_button_external_floor_1 = PB4
.equ pin_button_external_floor_2 = PB3
.equ pin_button_external_floor_3 = PB2

.equ pin_button_internal_floor_0 = PB1
.equ pin_button_internal_floor_1 = PB0
.equ pin_button_internal_floor_2 = PD7
.equ pin_button_internal_floor_3 = PD6

.equ pin_button_open_door = PD5
.equ pin_button_close_door = PD4

.equ pin_buzzer = PC5
.equ pin_led = PC4

/* -------------------------Definição dos pinos usados----------------------- */
/* -------------------------------------------------------------------------- */
/* -----------------------Definindo a fila das chamadas---------------------- */

.dseg
queue: .BYTE 4

/* -----------------------Definindo a fila das chamadas---------------------- */
/* -------------------------------------------------------------------------- */
/* -----------------------Alterando a mem�ria para cseg---------------------- */

.cseg
.org 0

/* -----------------------Alterando a mem�ria para cseg---------------------- */
/* -------------------------------------------------------------------------- */
/* ------------------------Tabela de interrup��o (IVT)----------------------- */

rjmp SETUP
.org OC1Aaddr
jmp OCI1A_Interrupt

/* ------------------------Tabela de interrup��o (IVT)----------------------- */
/* -------------------------------------------------------------------------- */
/* --------------------------------Interrup��es------------------------------ */

OCI1A_Interrupt:
	push global_temp
	in global_temp, SREG
	push global_temp

	// Incrementando o contador
	inc counter

	pop global_temp
	out SREG, global_temp
	pop global_temp

	reti

/* --------------------------------Interrup��es------------------------------ */
/* -------------------------------------------------------------------------- */
/* -----------------------------------Fun��es-------------------------------- */

/* ---------Clear counter: Zera o contador e o registrador de contagem------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (temp)                                           */
/* Defini��o da fun��o: void Clear_counter()                                  */
Clear_counter:
	push global_temp

	// Zerando o contador
	clr counter

	// Zerando o registrador de contagem
	clr global_temp
	sts TCNT1L, global_temp
	sts TCNT1H, global_temp

	pop global_temp

	ret

/* ----------------Add Call: Adiciona uma nova chamada na fila--------------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16, r17, r18, r19                                   */
/* Defini��o da fun��o: void Add_call(floor int, location int)                */
/* Argumentos: floor (r18), location (r19)                                    */
Add_call:
	push r16
	push r17

	// Copia o valor do argumento floor (r18) para o registrador r17
	mov r17, r18

	// Aplica a fun��o (3 - x) para converter o andar em uma posi��o da fila
	ldi r16, 3
	sub r16, r17

	// Inicializa��o o ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Desloca o ponteiro para a posi��o do andar na fila
	add ZL, r16
	sbci ZH, 0

	// L� o dado armazenado na posi��o da fila e salva no registrador r17
	ld r17, Z

	// Salva no valor lido a chamada atual (interna: 0b10 ou externa: 0b01)
	// O operador "or" para armazenar as chamadas internas e externas
	or r17, r19

	// Salva o novo valor na posi��o da fila
	st Z, r17

	pop r17
	pop r16

	ret

/* -----------Clear Call: Limpa a chamada que foi executada da fila---------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16, r17                                             */
/* Defini��o da fun��o: void Clear_call()                                     */
Clear_call:
	push r16

	// Copia o valor do registrador floor para o registrador r17
	mov r17, global_current_floor

	// Aplica a fun��o (3 - x) para converter o andar em uma posi��o da fila
	ldi r16, 3
	sub r16, r17

	// Inicializa��o o ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Desloca o ponteiro para a posi��o do andar na fila
	add ZL, r16
	sbci ZH, 0

	// Realiza um clear no registrador r17
	clr r17

	// Salva o novo valor na posi��o da fila (limpa a posi��o da fila)
	st Z, r17

	pop r16

	ret

/* ---Get next floor: Atribui ao registrador next_floor o novo andar alvo---- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16, r17, r18, r19, r20                              */
/* Defini��o da fun��o: int Get_next_floor()                                 */
/* Retorno: r17 (andar com maior prioridade)                                  */
Get_high_priority_floor:
	push r16
	push r17
	push r18
	push r19

	// Copia o valor do registrador floor para o registrador r17
	mov r17, global_current_floor

	// Aplica a fun��o (3 - x) para converter o andar em uma posi��o da fila
	ldi r16, 3
	// Salvando no registrador r16 o andar (convertido para a posi��o na fila)
	sub r16, r17

	// Salvando o valor 3 no registrador r17 para converter a posi��o de retorno em um andar
	ldi r17, 3

	// Inicializa��o o ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Switch case para a vari�vel direction
	switch_direction:
		cpi direction, 0
		breq case_direction_stop
		cpi direction, 1
		breq case_direction_up
		cpi direction, -1
		breq case_direction_down

	case_direction_stop:
		// Inicializa o registrador r18 com zero (para realizar o loop de 0 a 3)
		clr r18
		while_direction_stop:
			// Verificar se j� passou por todos os valores
			cpi r18, 4
			brge end_while_direction_stop

			// L� a posi��o atual da fila
			ld r19, Z+ 

			// Verifica se existe uma chamada
			cpi r19, 0
			// Caso n�o exista ele pula o retorno
			breq skip_direction_stop

			// Convertendo a posi��o encontrada para um andar (3 - posi��o)
			sub r17, r18

			rjmp found_result

			skip_direction_stop:
				inc r18
				rjmp while_direction_stop
		end_while_direction_stop:

		rjmp end_switch_direction
	case_direction_up:
		// Desloca o ponteiro para a posi��o do andar na fila
		add ZL, r16
		sbci ZH, 0

		// Inicializa o registrador r18 com o valor do registrador 16 (andar atual na fila)
		mov r18, r16
		while_direction_up_internal:
			// Verificar se j� passou por todos os valores
			cpi r18, 0
			brlt end_while_direction_up_internal

			// L� a posi��o atual da fila
			ld r19, Z

			// Decrementa o ponteiro Z
			subi ZL, 1
			sbci ZH, 0

			// Faz uma m�scara no bit referente a chamada interna
			andi r19, 0b00000010
			// Verifica se existe uma chamada interna
			cpi r19, 0
			// Caso n�o exista ele pula o retorno
			breq skip_direction_up_internal

			// Convertendo a posi��o encontrada para um andar (3 - posi��o)
			sub r17, r18

			rjmp found_result

			skip_direction_up_internal:
				dec r18
				rjmp while_direction_up_internal
		end_while_direction_up_internal:

		// Reseta a posi��o do ponteiro Z
		ldi ZL, low(queue)
		ldi ZH, high(queue)

		// Inicializa o registrador r18 com o valor zero
		clr r18
		// Incrementa o valor de r16 em 1
		inc r16
		while_direction_up_external:
			// Verificar se j� passou por todos os valores
			cp r18, r16
			brge end_while_direction_up_external

			// L� a posi��o atual da fila
			ld r19, Z+

			// Faz uma m�scara no bit referente a chamada externa
			andi r19, 0b00000001
			// Verifica se existe uma chamada externa
			cpi r19, 0
			// Caso n�o exista ele pula o retorno
			breq skip_direction_up_external

			// Convertendo a posi��o encontrada para um andar (3 - posi��o)
			sub r17, r18

			rjmp found_result

			skip_direction_up_external:
				inc r18
				rjmp while_direction_up_external
		end_while_direction_up_external:

		rjmp end_switch_direction
	case_direction_down:
		// Desloca o ponteiro para a posi��o do andar na fila
		add ZL, r16
		sbci ZH, 0

		// Inicializa o registrador r18 com o valor do registrador 16 (andar atual na fila)
		mov r18, r16
		while_direction_down:
			// Verificar se j� passou por todos os valores
			cpi r18, 4
			brge end_while_direction_down

			// L� a posi��o atual da fila
			ld r19, Z+

			// Verifica se existe uma chamada
			cpi r19, 0
			// Caso n�o exista ele pula o retorno
			breq skip_direction_down

			// Convertendo a posi��o encontrada para um andar (3 - posi��o)
			sub r17, r18

			rjmp found_result

			skip_direction_down:
				inc r18
				rjmp while_direction_down
		end_while_direction_down:

		rjmp end_switch_direction
	end_switch_direction:
		
	ldi r17, -1

	found_result:

	pop r19
	pop r18
	pop r17
	pop r16

	ret

/* -------Update Display: Uma subrotina que atualiza o andar no display------ */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (temp)                                           */
Update_display:
	push global_temp

	// Copiando o registrador floor para o registrador temp
	mov global_temp, global_current_floor
	// Movendo o andar atual para a posi��o do pino 2 e 3 da Porta D
	lsl global_temp
	lsl global_temp

	// Escrevendo o novo valor do display na Porta D
	out PORTD, global_temp

	pop global_temp

	ret

/* -----------Delay 20ms: Uma subrotina que aciona um delay de 20ms---------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (temp), r17, r18                                 */
/* Assumindo que o clock � 16Mhz, para 20ms s�o necess�rios 320.000 ciclos    */
/* CLOCK (16.0e6) * 20ms = CLOCK (16.0e6) * 0.02 = 320.000                    */
Delay_20ms:
	push r16
	push r17
	push r18

	// Calculando a quantidade de ciclos que o loop abaixo deve ser executado
	// Como no loop cada itera��o vai consumir cinco ciclos, precisamos dividir os 320.000 ciclos por cinco
	.equ required_cycles = int(CLOCK * 0.02 / 5.0)

	// Salvando os valores em tr�s registradores
	ldi r18, byte3(required_cycles)
	ldi r17, high(required_cycles)
	ldi r16, low(required_cycles)

	// Executando os 320.000 cliclos
	subi r16, 1
	sbci r17, 0
	sbci r18, 0
	brcc pc-3

	pop r18
	pop r17
	pop r16

	ret

/* -----------------------------------Fun��es-------------------------------- */

SETUP:
	/* ------------------------Inicializa��o da pilha------------------------ */

	ldi global_temp, high(RAMEND)
	out SPH, global_temp
	ldi global_temp, low(RAMEND)
	out SPL, global_temp

	/* ------------------------Inicializa��o da pilha------------------------ */
	/* ---------------------------------------------------------------------- */
	/* ---------------------Defini��o das portas B, D e C-------------------- */

	/* Porta B                                                                */
	/*                                                                        */
	/* PB5 = Andar 0 (Bot�es externos) - Entrada                              */
	/* PB4 = Andar 1 (Bot�es externos) - Entrada                              */
	/* PB3 = Andar 2 (Bot�es externos) - Entrada                              */
	/* PB2 = Andar 3 (Bot�es externos) - Entrada                              */
	/* PB1 = Andar 0 (Bot�es internos) - Entrada                              */
	/* PB0 = Andar 1 (Bot�es internos) - Entrada                              */
	ldi global_temp, 0b00000000

	// Configurando a Porta B
	out DDRB, global_temp

	/* Porta D                                                                */
	/*                                                                        */
	/* PD7 = Andar 2 (Bot�es internos) - Entrada                              */
	/* PD6 = Andar 3 (Bot�es internos) - Entrada                              */
	/* PD5 = Abrir Porta - Entrada                                            */
	/* PD4 = Fechar Porta - Entrada                                           */
	/* PD3 = Display 7 segmentos (Entrada B) - Sa�da                          */
	/* PD2 = Display 7 segmentos (Entrada A) - Sa�da                          */
	ldi global_temp, 0b00001100

	// Configurando a Porta D
	out DDRD, global_temp

	/* Porta C                                                                */
	/*                                                                        */
	/* PC5 = Buzzer - Sa�da                                                   */
	/* PC4 = Led - Sa�da                                                      */
	ldi global_temp, 0b00110000

	// Configurando a Porta C
	out DDRC, global_temp

	/* ---------------------Defini��o das portas B, D e C-------------------- */
	/* ---------------------------------------------------------------------- */
	/* -------------Definindo o estado inicial dos pinos de Sa�da------------ */

	/* Porta C                                                                */
	/*                                                                        */
	/* PC5 = Buzzer - 0                                                       */
	/* PC4 = Led - 0                                                          */
	cbi PORTC, pin_buzzer
	cbi PORTC, pin_led

	/* -------------Definindo o estado inicial dos pinos de Sa�da------------ */
	/* ---------------------------------------------------------------------- */
	/* --------------Definindo o estado inicial dos registradores------------ */

	// Atribuindo os valores aos registradores
	ldi global_state, 1
	ldi global_current_floor, 0
	ldi next_floor, -1
	ldi direction, 0
	ldi counter, 0

	// Atualizando o display para o andar inicial
	call Update_display

	/* --------------Definindo o estado inicial dos registradores------------ */
	/* ---------------------------------------------------------------------- */
	/* -------------------------Inicializando a fila------------------------- */

	ldi global_temp, 0

	sts queue, global_temp
	sts queue + 1, global_temp
	sts queue + 2, global_temp
	sts queue + 3, global_temp

	/* -------------------------Inicializando a fila------------------------- */
	/* ---------------------------------------------------------------------- */
	/* ------------------------Configura��o do Timer A----------------------- */
	
	// Calculando o valor para o TOP
	.equ TOP = int(0.5 + ((CLOCK / PRESCALE_DIV_TIMER_A) * DELAY_TIMER_A))

	// Validando se ele est� dentro do limite esperado (65.532)
	.if TOP > 65535
	.error "TOP is out of range"
	.endif

	// Atribuindo o valor de compara��o (TOP)
	ldi global_temp, high(TOP)
	sts OCR1AH, global_temp
	ldi global_temp, low(TOP)
	sts OCR1AL, global_temp

	// Atribuindo o valor para o modo de opera��o (WGM)
	ldi global_temp, ((WGM_TIMER_A & 0b11) << WGM10)
	sts TCCR1A, global_temp
	ldi global_temp, ((WGM_TIMER_A >> 2) << WGM12) | (PRESCALE_TIMER_A << CS10)
	sts TCCR1B, global_temp

	// Habilita o bit na posi��o OCIE1A
	lds global_temp, TIMSK1
	sbr global_temp, 1 << OCIE1A
	sts TIMSK1, global_temp

	sei

	/* ------------------------Configura��o do Timer A----------------------- */

	rjmp MAIN

MAIN:
	/* ----------------Leitura dos bot�es (internos e externos)-------------- */
	sbic PINB, pin_button_external_floor_0
	rjmp button_external_floor_0_pressed

	sbic PINB, pin_button_external_floor_1
	rjmp button_external_floor_1_pressed

	sbic PINB, pin_button_external_floor_2
	rjmp button_external_floor_2_pressed

	sbic PINB, pin_button_external_floor_3
	rjmp button_external_floor_3_pressed

	sbic PINB, pin_button_internal_floor_0
	rjmp button_internal_floor_0_pressed

	sbic PINB, pin_button_internal_floor_1
	rjmp button_internal_floor_1_pressed

	sbic PIND, pin_button_internal_floor_2
	rjmp button_internal_floor_2_pressed

	sbic PIND, pin_button_internal_floor_3
	rjmp button_internal_floor_3_pressed
	/* ----------------Leitura dos bot�es (internos e externos)-------------- */

	rjmp switch_state

	/* -------Preparando os atributos para executar a fun��o (Add_call)------ */
	button_external_floor_0_pressed:
		ldi r18, 0
		ldi r19, 1
		
		rjmp button_pressed

	button_external_floor_1_pressed:
		ldi r18, 1
		ldi r19, 1
		
		rjmp button_pressed

	button_external_floor_2_pressed:
		ldi r18, 2
		ldi r19, 1
		
		rjmp button_pressed

	button_external_floor_3_pressed:
		ldi r18, 3
		ldi r19, 1
		
		rjmp button_pressed

	button_internal_floor_0_pressed:
		ldi r18, 0
		ldi r19, 2
		
		rjmp button_pressed

	button_internal_floor_1_pressed:
		ldi r18, 1
		ldi r19, 2
		
		rjmp button_pressed

	button_internal_floor_2_pressed:
		ldi r18, 2
		ldi r19, 2
		
		rjmp button_pressed

	button_internal_floor_3_pressed:
		ldi r18, 3
		ldi r19, 2
		
		rjmp button_pressed

	button_pressed:
		call Add_call
		call Delay_20ms
	/* -------Preparando os atributos para executar a fun��o (Add_call)------ */
	/* ---------------------------------------------------------------------- */
	/* -------------------------In�cio switch(estado)------------------------ */
	switch_state:
		cpi global_state, 1
		breq case_idle
		cpi global_state, 2
		breq case_elevator_ups
		cpi global_state, 3
		breq case_elevator_downs
		cpi global_state, 4
		breq case_new_floor
		cpi global_state, 5
		breq case_elevator_stops
		cpi global_state, 6
		breq case_buzzer_warning
		cpi global_state, 7
		breq case_holding_door
		cpi global_state, 8
		breq case_next_destiny

		case_idle:
			rjmp case_idle_longjump
		case_elevator_ups:
			rjmp case_elevator_ups_longjump
		case_elevator_downs:
			rjmp case_elevator_downs_longjump
		case_new_floor:
			rjmp case_new_floor_longjump
		case_elevator_stops:
			rjmp case_elevator_stops_longjump
		case_buzzer_warning:
			rjmp case_buzzer_warning_longjump
		case_holding_door:
			rjmp case_holding_door_longjump
		case_next_destiny:
			rjmp case_next_destiny_longjump

		/* -------------------------------Cases------------------------------ */
		case_idle_longjump:
			// Verificando se o bot�o de abrir a porta est� sendo pressionado
			sbic PIND, pin_button_open_door
			rjmp call_in_current_floor

			call Get_next_floor

			mov next_floor, r17

			// Se n�o tiver chamada, ir para "no_call"
			cpi next_floor, -1
			breq no_call

			// Se a chamada for no andar atual, ir para "call_in_current_floor"
			cp next_floor, global_current_floor
			breq call_in_current_floor

			// Se a chamada for para subir, ir para "call_up"
			cp next_floor, global_current_floor
			brge call_up

			call_down:
				ldi direction, -1
				// Mudando o estado para "case_elevator_downs"
				ldi global_state, 3
				// Resetando o contador
				call Clear_counter

				rjmp end_switch_state

			call_up:
				ldi direction, 1
				// Mudando o estado para "case_elevator_ups"
				ldi global_state, 2
				// Resetando o contador
				call Clear_counter

				rjmp end_switch_state

			call_in_current_floor:
				ldi direction, 0
				// Mudando o estado para "case_elevator_stops"
				ldi global_state, 5
				// Resetando o contador
				call Clear_counter

				rjmp end_switch_state

			no_call:
				ldi direction, 0

			rjmp end_switch_state
		case_elevator_ups_longjump:
			// Verificando se j� se passaram 3s
			cpi counter, 3
			brne wait_elevator_ups_done

			// Incrementando o andar atual
			inc global_current_floor
			// Atualizando o valor do display
			call Update_display

			// Mudando o estado para "case_new_floor"
			ldi global_state, 4
			// Resetando o contador
			call Clear_counter

			wait_elevator_ups_done:

			rjmp end_switch_state
		case_elevator_downs_longjump:
			// Verificando se j� se passaram 3s
			cpi counter, 3
			brne wait_elevator_downs_done

			// Decrementando o andar atual
			dec global_current_floor
			// Atualizando o valor do display
			call Update_display

			// Mudando o estado para "case_new_floor"
			ldi global_state, 4
			// Resetando o contador
			call Clear_counter

			wait_elevator_downs_done:

			rjmp end_switch_state
		case_new_floor_longjump:
			call Get_next_floor

			// Verificando se esse é o andar com maior prioridade
			cp global_current_floor, r17
			breq stop_elevator

			// Caso n�o seja o andar alvo, voltar para "case_idle"
			ldi global_state, 1

			rjmp end_switch_state

			stop_elevator:
				// Caso seja o andar alvo, ir para o pr�ximo estado "case_elevator_stops"
				ldi global_state, 5

			rjmp end_switch_state
		case_elevator_stops_longjump:
			// Verificando se o bot�o de fechar a porta foi pressionado
			sbis PIND, pin_button_close_door
			rjmp skip_close_door_a

			// Mudando o estado para "case_next_destiny"
			ldi global_state, 8
			// Resetando o contador
			call Clear_counter

			rjmp end_switch_state

			skip_close_door_a:
				// Ativando o led
				sbi PORTC, pin_led

				// Verificando se j� se passaram 5s
				cpi counter, 5
				brne end_switch_state

				// Mudando o estado para "case_buzzer_warning"
				ldi global_state, 6
				// Resetando o contador
				call Clear_counter

			rjmp end_switch_state
		case_buzzer_warning_longjump:
			// Verificando se o bot�o de fechar a porta foi pressionado
			sbis PIND, pin_button_close_door
			rjmp skip_close_door_b

			// Mudando o estado para "case_next_destiny"
			ldi global_state, 8
			// Resetando o contador
			call Clear_counter

			rjmp end_switch_state

			skip_close_door_b:
				// Ativando o buzzer
				sbi PORTC, pin_buzzer

				// Verificando se j� se passaram 5s
				cpi counter, 5
				brne end_switch_state

				// Mudando o estado para "case_holding_door"
				ldi global_state, 7
				// Resetando o contador
				call Clear_counter

			rjmp end_switch_state
		case_holding_door_longjump:
			// Verificando se o bot�o de abrir a porta est� sendo pressionado
			sbis PIND, pin_button_open_door
			rjmp close_door
		
			// Aplicando um delay para evitar o bounce
			call Delay_20ms

			rjmp end_switch_state

			close_door:
				// Mudando estado para "case_next_destiny"
				ldi global_state, 8

			rjmp end_switch_state
		case_next_destiny_longjump:
			// Desativando o buzzer e o led
			cbi PORTC, pin_buzzer
			cbi PORTC, pin_led

			// Removendo a chamada da fila
			call Clear_call

			// Mudando estado para "case_idle"
			ldi global_state, 1

			rjmp end_switch_state
		end_switch_state:
		/* -------------------------------Cases------------------------------ */
	rjmp MAIN