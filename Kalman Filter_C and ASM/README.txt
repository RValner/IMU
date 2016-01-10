Igaks juhuks väike readme

Tegu on kalman filtri implementatsiooniga. Antud juhul kasutatakse
testkoodis kiirendusanduri ja güroskoobi väljundandmeid ning arvutatakse
nende põhjal filtrit rakendava seadme asend.

Testkoodi sisendiks on varasemalt ära salvestatud kalman filtrit
rakendava seadme väljundandmed txt faili kujul. Väljundandmetes
on 3 andmevälja: Kiirendusanduri andmed, güroskoobi andmed ja
nende põhjal jooksutatud kalman filtri tulemus ajasammude lõikes.

Testkood "kalmanTest.c" väljastab need 3 andmevälja + assembleris
implementeeritud kalman filtri põhifunktsiooni tulemused. Koodi
eduka töö tunnus oleks see, et arvutatud tulemused ja varasemalt
mõõdetud tulemused langevad kokku. Sisendandmete genereerimisel
esineb mõningaseid ümardusvigasid ning seeläbi võivad tulemused
erineda +/- 0.005 võrra. Samuti ei salvestanud ma ära algandmete
genereerimisel filtri lähteasendi ning seetõttu läheb testkoodis
filtri stabiliseerimiseks "veidi aega".

Programmi käivitamine:

	käsurealt: make all
	käsurealt: kalmanTest.exe ".\Sample Data\sampleData.txt"

Tõestus, et programm tegi midagi: pilt "pictures" kaustas.
Kõiksugu muu jutt tuleb ette programmi käivitades. Osa teksti
ka kalmanTest.c preambulas

Kirjutasin enamasti inglise keeles kuna mul on tavaks panna
kõiksugu vähegi huvitav ja kellelegi kasulik blogisse üles.
