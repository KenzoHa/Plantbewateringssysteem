# Plantbewateringssysteem
Schoolopdracht: 

Een klant wil de mogelijkheid bekijken om in zijn serre de bewatering van zijn planten automatisch per plant te sturen. Je implementeert in dit project een automatisch bewateringssysteem voor één plant.
Via de juiste sensoren moet de bodemvochtigheid en de buitentemperatuur worden gemeten. De bodemvochtigheidssensoren moeten gecalibreerd worden met minimum 3 statussen: droog, vochtig en nat. Wanneer de bodem droog is moet de plant extra water krijgen tot de bodem vochtig is (dit mag in meerdere keren).  Wanneer de buitentemperatuur hoger is dan 25°C, moet er meer water bijgegeven worden.  Als het kouder is dan 5°C mag er géén water gegeven worden.
Er moet ook een panic button toegevoegd worden waarbij er altijd water wordt gegeven wanneer op de knop wordt gedrukt.  De hoeveelheid water die gegeven wordt moet via de software gewijzigd kunnen worden.

Kwaliteitscriteria:

Je maakt gebruik van 2 verschillende bodemvochtigheidssensoren én een temperatuursensor;
Je calibreert de 2 bodemvochtigheidssensoren met minimum 3 statussen (droog, vochtig, nat) en maakt in de code duidelijk hoe je de gecalibreerde intervallen bepaald hebt;
Je gebruikt een relais om de pomp aan te sturen;
Je implementeert een panic button die altijd water geeft maar die softwarematig bepaalt voor hoe lang;
Je mag gebruik maken van een breadboard, solderen is nog niet nodig (we hebben nog andere componenten nodig nadien)
Je implementeert de flowchart die we samen in de klas gemaakt hebben voor de aansturing van de pomp;
Je gebruikt een library om daarin een aantal debug statements toe te voegen die je kan aan/uitzetten en die je via een seriële communicatie informatie geven over de werking van je systeem;
Je past de coding guidelines toe in je code, en bovendien:
bevat je code comments waarin je:
verwijst naar gebruikte documentatie
beargumenteert waarom je beslissingen hebt genomen
bevat je code functies, zowel voor specifieke taken of om de leesbaarheid van de code te verhogen
gebruik je een aparte configuratie header file om configuratiegegevens in op te slaan (bv: calibratiegegevens, temperatuurgegevens, ...)
Je maakt gebruik van het testdocument dat we samen in de klas hebben opgesteld om te laten zien welke testen je uitgevoerd hebt.
