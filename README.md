# LSM6DS3 - Driver

<h2>Utilisation de l'accéléromètre 3D</h2>

<p>L'utilisation de l'accéléromètre seul suffit à notre application, le gyroscope ne sera donc pas utilisé. Dans un soucis d'optimisation de la consommation énergétique, nous utiliserons le LSM6DS3 en mode "accelerometer only". Le LSM6DS3 a 3 modes de fonctionnement :

<ul>
  <li><b>Accéléromètre en fonction, gyroscope éteint</b></li>
  <li>Gyroscope en fonction, accéléromètre éteint</li>
  <li>Accéléromètre et gyroscope en fonction avec Output Data Rate (ODR) indépendants</li>
</ul>
Etant donné les besoins de notre application, l'utilisation de l'accéromètre seul suffit, dans un soucis d'optimisation de la consommation énergétique, nous utiliserons le LSM6DS3 en mode "Accelerometer only".</p>

</p>Il existe quelque soit la configuration précédemment choisie, plusieurs modes de consommation
<ul>
  <li><b>Low power</b></li>
  <li>Normal</li>
  <li>Hautes performances</li>
</ul>
Nous utiliserons le mode low power</p>
<p><img src="datasheet/signal.png" alt="hi" class="inline"/> 
</p>

<h3>Configuration</h3>
<ol>
  <li>Accéléromètre en mode low power</li>
  <li>ODR : 52 Hz</li>
  <li>Consommation électrique : 45 µA</li>  
</ol>
