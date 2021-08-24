/*
 * __Example_Music.cpp
 *
 *  Created on: 2021年8月22日
 *      Author: BillY
 */


#include "TaskConfig.h"

#if INCLUDE_Example_Music

#include "__Example_Music.h"
#include "Buzzer.h"
#include "Button.h"

#define INCLUDE_Mario               1
#define INCLUDE_War_Cant_of_Mars    0

extern Module::Buzzer* Buzzer;
extern Module::Button* Button;

typedef Module::BuzzerNote Note;

#if INCLUDE_Mario
static const Module::BuzzerNoteDelayed Mario[] = {
    {Note::Mi3M, 80}, {Note::Silent, 80},  {Note::Mi3M, 80}, {Note::Silent, 240},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
    {Note::So5L, 80}, {Note::Silent, 0},   {Note::Finish, 0}
};
#endif

#if INCLUDE_War_Cant_of_Mars
static const Module::BuzzerNoteDelayed War_Cant_of_Mars[] = {
//	{Note::So5M, 400},   {Note::So5M, 200},  {Note::So5M, 200},
//	{Note::So5M, 400},   {Note::Fa4M, 200},  {Note::Mi3M, 400},
//	{Note::So5M, 200},   {Note::Do1H, 400},  {Note::Re2H, 200},
//	{Note::Mi3H, 400},   {Note::Mi3H, 200},  {Note::Mi3H, 400},
//	{Note::Re2H, 200},   {Note::Do1H, 400},  {Note::Do1H, 400},
//	{Note::Si7M, 200},   {Note::La6M, 400},  {Note::La6M, 200},
//	{Note::La6M, 400},   {Note::Si7M, 200},  {Note::Do1H, 400},
//	{Note::Si7M, 200},   {Note::Do1H, 400},  {Note::La6M, 200},
//	{Note::So5M, 400},   {Note::La6M, 200},  {Note::So5M, 400},
//	{Note::Mi3M, 200},   {Note::So5M, 800},  {Note::So5M, 400},
//	{Note::So5M, 200},   {Note::So5M, 400},  {Note::So5M, 200},
//	{Note::So5M, 400},   {Note::Fa4M, 200},  {Note::Mi3M, 400},
//	{Note::So5M, 200},   {Note::Do1H, 400},  {Note::Re2H, 200},
//	{Note::Mi3H, 400},   {Note::Mi3H, 200},  {Note::Mi3H, 400},
//	{Note::Re2H, 200},   {Note::Do1H, 800},  {Note::Do1H, 800},
//	{Note::Re2H, 800},   {Note::Re2H, 800},  {Note::Do1H, 800},
//	{Note::Si7M, 800},   {Note::Do1H, 1600}, {Note::Silent, 400},
    {Note::Silent, 400},
    {Note::So5M, 800},
    {Note::Fa4M, 400},
    {Note::Mi3M, 400},
    {Note::So5M, 200},
    {Note::Do1H, 400},
    {Note::Re2H, 200},
    {Note::Mi3H, 1200},
    {Note::Do1H, 800},
//	{Note::Silent, 400}, {Note::La6M, 800},   {Note::Si7M, 400},
//	{Note::Do1H, 400},   {Note::Si7M, 200},   {Note::Do1H, 400},
//	{Note::La6M, 200},   {Note::So5M, 1600},  {Note::Mi3M, 800},
//	{Note::Silent, 400}, {Note::So5M, 800},   {Note::Fa4M, 400},
//	{Note::Mi3M, 400},   {Note::So5M, 200},   {Note::Do1H, 400},
//	{Note::Re2H, 200},   {Note::Mi3H, 1600},  {Note::Do1H, 800},
//	{Note::Do1H, 800},   {Note::Re2H, 800},   {Note::Re2H, 800},
//	{Note::Do1H, 800},   {Note::Si7M, 800},   {Note::Do1H, 1600},
    {Note::Silent, 0},
    {Note::Finish, 0}
};
#endif

static bool press = false;

__NO_RETURN void Example_Music(void* argument) {
	Buzzer_Init();
	Button_Init();
	while (true) {
		if (press) {
#if INCLUDE_Mario
			Buzzer->SingSong(Mario);
#endif
#if INCLUDE_War_Cant_of_Mars
			Buzzer->SingSong(War_Cant_of_Mars);
#endif
			press = false;
		}
		osDelay(50);
	}
}

void Module::Button::IntCallback() {
	press = true;
}

#endif

