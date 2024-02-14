const pautasGuardadas = localStorage.getItem("pautasDiarias");

// Si hay datos almacenados en localStorage, úsalos, de lo contrario, utiliza el array predeterminado
const pautasDiarias = pautasGuardadas
  ? JSON.parse(pautasGuardadas)
  : [
      { Id: "Pauta 1", Dueño: "Juan", Mascota: "Max", Confirma: true },
      { Id: "Pauta 2", Dueño: "Maria", Mascota: "Bella", Confirma: false },
      { Id: "Pauta 3", Dueño: "Luis", Mascota: "Rocky", Confirma: true },
      { Id: "Pauta 4", Dueño: "Ana", Mascota: "Coco", Confirma: true },
    ];

// Restaurar los datos en localStorage después de agregar una nueva cita
localStorage.setItem("pautasDiarias", JSON.stringify(pautasDiarias));

// Declaramos una función para el registro de usuarios nuevos

function agregarNuevoDueño() {
  console.log("Entrando a agregarNuevoDueño...");
  const nuevoDueño = document.getElementById("nuevoDueño").value;
  const nuevaMascota = document.getElementById("nuevaMascota").value;

  // Validación de datos
  if (!nuevoDueño || !nuevaMascota) {
    // Mostrar mensaje de error
    Toastify({
      text: "Debe completar todos los campos.",
      duration: 3000,
      close: true,
      gravity: "bottom",
      position: "right",
      className: "dark",
    }).showToast();
    return;
  }

  // Utilizar SweetAlert2 para obtener la confirmación del usuario
  Swal.fire({
    title: "¿Reservará una cita?",
    icon: "question",
    showCancelButton: true,
    confirmButtonText: "Sí",
    cancelButtonText: "No",
  }).then((result) => {
    if (result.isConfirmed) {
      // Código para el caso "Sí"
      const nuevoId = "Pauta " + (pautasDiarias.length + 1);
      const nuevoEnlace = "https://dog.ceo/api/breeds/image/random"; // Enlace fijo para cada nueva cita
      const nuevaCita = {
        Id: nuevoId,
        Dueño: nuevoDueño,
        Mascota: nuevaMascota,
        Confirma: true,
        Enlace: nuevoEnlace,
      };

      pautasDiarias.push(nuevaCita);

      // Agregar la nueva cita en localStorage
      localStorage.setItem("pautasDiarias", JSON.stringify(pautasDiarias));

      console.log("Nueva cita agregada:", nuevaCita);
      limpiarContenedor();
      mostrarCitaEnHTML(nuevaCita);

      // Mostrar mensaje de usuario registrado y contador
      mostrarMensaje(
        "Usuario registrado. La página se recargará automáticamente..."
      );

      let segundosRestantes = 5;
      const contador = setInterval(() => {
        segundosRestantes--;
        mostrarMensaje(
          `Tiene ${segundosRestantes} segundos para editar su asistencia.`
        );

        // Detener el contador si el usuario hace clic en "Editar Confirmación"
        const editarBtn = document.createElement("button");
        editarBtn.textContent = "Editar Confirmación";
        editarBtn.classList.add("editar-btnEditar");
        editarBtn.addEventListener("click", () => {
          clearInterval(contador);
          mostrarMensaje("Puede editar la confirmación de la cita");
        });

        // Agregar botón de editar confirmación al contenedor del mensaje
        mensajeContainer.appendChild(editarBtn);

        // Recargar la página después de 10 segundos
        if (segundosRestantes === 0) {
          clearInterval(contador);
          location.reload();
        }
      }, 5000);
    } else {
      Swal.fire({
        title: "Registro pendiente",
        text: "Su registro de cita está pendiente. ¡Gracias!",
        icon: "info",
      }).then((result) => {
        if (result.isConfirmed) {
          location.reload();
        }
      });
    }
  });
}

function mostrarMensaje(mensaje) {
  // Limpiar el contenedor del mensaje
  mensajeContainer.innerHTML = "";

  // Crear elemento de mensaje y mostrar en pantalla
  const mensajeElement = document.createElement("div");
  mensajeElement.textContent = mensaje;
  mensajeContainer.appendChild(mensajeElement);
}

// Esta función muestra la cita registrada en el HTML

function mostrarCitaEnHTML(cita) {
  const container = document.getElementById("citasContainer");

  if (!cita) {
    container.innerHTML = "Cita no encontrada";
    return;
  }

  // Crear contenedor div para cada resultado
  const resultadoContainer = document.createElement("div");

  // Crear elementos HTML
  const h2 = document.createElement("h2");
  const idParrafo = document.createElement("p");
  const dueñoParrafo = document.createElement("p");
  const mascotaParrafo = document.createElement("p");
  const confirmadaParrafo = document.createElement("p");
  const imagenMascotaFetch = document.createElement("div");

  // Crear contenedor para el enlace y el botón
  const enlaceContainer = document.createElement("div");

  // Agregar clases de estilo
  h2.classList.add("inline-block-element");
  idParrafo.classList.add("inline-block-element");
  dueñoParrafo.classList.add("inline-block-element");
  mascotaParrafo.classList.add("inline-block-element");
  confirmadaParrafo.classList.add("inline-block-element");
  imagenMascotaFetch.classList.add("inline-block-element");
  enlaceContainer.classList.add("inline-block-element");
  resultadoContainer.classList.add("resultado-container");

  // Asignar contenido al enlaceContainer
  const enlaceBtn = document.createElement("button");
  enlaceBtn.textContent = "Adjuntar Imagen";
  enlaceBtn.classList.add("enlace-btn");
  enlaceBtn.addEventListener("click", function () {
    // Realizar un nuevo fetch al hacer clic en el botón
    fetch(cita.Enlace)
      .then((response) => response.json())
      .then((data) => {
        const imgElement = document.createElement("img");
        imgElement.src = data.message;
        imgElement.alt = "Imagen de Perro";

        // Agregar la imagen al div después de que se haya cargado
        imgElement.onload = function () {
          imagenMascotaFetch.innerHTML = "";
          imagenMascotaFetch.appendChild(imgElement);
        };
      })
      .catch((error) => {
        console.error("Error al obtener la imagen:", error);
      });
  });

  // Añadir botón de enlace al contenedor del enlace
  enlaceContainer.appendChild(enlaceBtn);

  // Realizar fetch para obtener el enlace
  fetch("https://dog.ceo/api/breeds/image/random")
    .then((response) => response.json())
    .then((data) => {
      // Agregar el enlace al contenedor
      const enlace = document.createElement("a");
      enlace.href = data.message;

      // Agregar el enlace al contenedor del enlace
      enlaceContainer.appendChild(enlace);
    })
    .catch((error) => {
      console.error("Error al obtener el enlace:", error);
    });

  // Agregar estilo con white-space a los elementos
  h2.style.whiteSpace = "pre-line";
  idParrafo.style.whiteSpace = "pre-line";
  dueñoParrafo.style.whiteSpace = "pre-line";
  mascotaParrafo.style.whiteSpace = "pre-line";
  confirmadaParrafo.style.whiteSpace = "pre-line";
  imagenMascotaFetch.style.whiteSpace = "pre-line";

  // Asignar contenido a los elementos
  h2.textContent = "Cita";
  idParrafo.textContent = `ID: ${cita.Id}`;
  dueñoParrafo.textContent = `Dueño: ${cita.Dueño}`;
  mascotaParrafo.textContent = `Mascota: ${cita.Mascota}`;
  confirmadaParrafo.textContent = "Confirma: ";

  const confirmacionSpan = document.createElement("span");
  confirmacionSpan.textContent = cita.Confirma ? "Sí" : "No";
  confirmacionSpan.classList.add(
    cita.Confirma ? "confirmada" : "no-confirmada"
  );
  confirmadaParrafo.appendChild(confirmacionSpan);

  // Añadir div de la imagen al contenedor
  resultadoContainer.appendChild(imagenMascotaFetch);

  // Aplicar la clase confirmada si la cita está confirmada
  if (cita.Confirma) {
    confirmadaParrafo.classList.add("confirmada");
  }

  // Añadir elementos al contenedor
  container.appendChild(h2);
  container.appendChild(idParrafo);
  container.appendChild(dueñoParrafo);
  container.appendChild(mascotaParrafo);
  container.appendChild(confirmadaParrafo);

  // Agregar contenedor del enlace al contenedor principal
  container.appendChild(enlaceContainer);

  // Añadir contenedor div al contenedor principal
  container.appendChild(resultadoContainer);

  // Agregar botón de editar
  const editarBtn = document.createElement("button");
  editarBtn.textContent = "Modificar cita";
  editarBtn.classList.add("editar-btn");
  editarBtn.addEventListener("click", function () {
    editarCita(cita.Id);
  });

  // Añadir botón de editar al contenedor
  resultadoContainer.appendChild(editarBtn);
}

function editarCita(id) {
  const cita = pautasDiarias.find((c) => c.Id === id);

  if (cita) {
    // Mostrar un mensaje de confirmación con SweetAlert
    Swal.fire({
      title: "Cambiar estado de confirmación",
      text: "¿Desea cambiar el estado de confirmación?",
      icon: "question",
      showCancelButton: true,
      confirmButtonText: "Sí",
      cancelButtonText: "No",
    }).then((result) => {
      if (result.isConfirmed) {
        // Resto del código para cambiar el estado de confirmación
        cita.Confirma = !cita.Confirma;

        // Actualizar la visualización en el HTML
        limpiarContenedor();
        mostrarCitaEnHTML(cita);

        // Actualizar en sessionStorage
        sessionStorage.setItem(id, JSON.stringify(cita));

        // Mostrar mensaje de éxito con Toastify
        Toastify({
          text: "Cita modificada con éxito!",
          duration: 60000,
          position: "center",
          backgroundColor: "rgb(9, 26, 159)",
          className: "toastify",
          stopOnFocus: false,
          offset: {
            x: 0, // Centrado horizontalmente
            y: 500, // Centrado verticalmente
          },
        }).showToast();

        // Mostrar cambios en el console.log después de 5 segundos
        setTimeout(() => {
          console.log(
            `Cita ${id} editada: Confirmación ahora es ${
              cita.Confirma ? "Sí" : "No"
            }`
          );
          console.log("Nuevo estado en pautasDiarias:", pautasDiarias);
          console.log(
            "Nuevo estado en sessionStorage:",
            JSON.parse(sessionStorage.getItem(id))
          );

          // Realizar un reload después de 6 segundos
          location.reload();
        }, 6000);
      }
    });
  } else {
    console.log("cita no modificada");
    Toastify({
      text: "Cita no encontrada o modificada.",
      duration: 3000,
      position: "center",
      backgroundColor: "#FF6347", // Color de fondo rojo
      className: "toastify",
      stopOnFocus: true,
      offset: {
        x: 0, // Centrado horizontalmente
        y: 500, // Centrado verticalmente
      },
    }).showToast();
  }
}


// Luego, declaramos las funciones de búsqueda según recomendaciones de buenas prácticas (ChatGpt). Agregamos el método .toLowerCase() para que no hayan problemas en las entradas de datos del usuario

function buscarCitaPorCampo(campo, valor, mensajeNoEncontrado) {
  limpiarContenedor(); // Limpiar el contenedor
  const citaEncontrada = pautasDiarias.find(
    (item) => item[campo].toLowerCase() === valor.toLowerCase()
  );

  if (citaEncontrada) {
    mostrarCitaEnHTML(citaEncontrada);
  } else {
    const agregarNuevoDueñoConfirm = confirm(mensajeNoEncontrado);
    if (agregarNuevoDueñoConfirm) {
      agregarNuevoDueño();
    } else {
      mostrarMensaje(`Se ha cancelado el registro de la ${campo}.`);
    }
  }

  // Restaurar la visibilidad del campo de entrada
  if (campo === "Mascota") {
    document.getElementById("nuevoDueño").style.display = "block";
  } else if (campo === "Dueño") {
    document.getElementById("nuevaMascota").style.display = "block";
  }
}

function buscarCitaPorMascota(mascota) {
  // Mostrar campo de nombre de la mascota
  document.getElementById("nuevaMascota").style.display = "block";
  // Ocultar campo de nombre del dueño
  document.getElementById("nuevoDueño").style.display = "none";

  buscarCitaPorCampo(
    "Mascota",
    mascota,
    "No se encontró la mascota. ¿Desea agregar un nuevo dueño?"
  );
}

function buscarCitaPorDueño(dueño) {
  // Mostrar campo de nombre del dueño
  document.getElementById("nuevoDueño").style.display = "block";
  // Ocultar campo de nombre de la mascota
  document.getElementById("nuevaMascota").style.display = "none";

  buscarCitaPorCampo(
    "Dueño",
    dueño,
    "No se encontró el dueño. ¿Desea agregar un nuevo dueño?"
  );
}

function buscarCitaPorPauta(pauta) {
  limpiarContenedor(); // Limpiar el contenedor
  const citaEncontrada = pautasDiarias.find((item) => item.Id === pauta);
  mostrarResultadoBusqueda(citaEncontrada);
}

function limpiarContenedor() {
  const container = document.getElementById("citasContainer");
  container.innerHTML = ""; // Establecer el contenido en una cadena vacía
}

// Código principal

function mostrarTodasLasCitas() {
  limpiarContenedor(); // Limpiar el contenedor antes de mostrar las citas

  // Itera sobre todas las citas y muestra cada una
  pautasDiarias.forEach((pautasDiarias) => {
    mostrarCitaEnHTML(pautasDiarias);
  });
}

function procesarFormulario() {
  console.log("Procesando formulario...");

  const tipoUsuario = document.getElementById("tipoUsuario").value;
  console.log("Tipo de usuario:", tipoUsuario);

  if (tipoUsuario === "dueño") {
    limpiarContenedor();
    const accionDueño = document.getElementById("accionDueño").value;
    console.log("Acción dueño:", accionDueño);

    if (accionDueño === "buscarMascota") {
      const mascotaABuscar = document.getElementById("nuevaMascota").value;
      console.log("Mascota a buscar:", mascotaABuscar);
      buscarCitaPorMascota(mascotaABuscar);
    } else if (accionDueño === "agregarUsuario") {
      agregarNuevoDueño();
    }
  } else if (tipoUsuario === "groomer") {
    limpiarContenedor();
    const accionGroomer = document.getElementById("accionGroomer").value;
    console.log("Acción groomer:", accionGroomer);

    if (accionGroomer === "mostrarTodas") {
      limpiarContenedor();
      mostrarTodasLasCitas();
    } else if (accionGroomer === "buscarMascotaGroomer") {
      const mascotaABuscarGroomer = document.getElementById(
        "mascotaABuscarGroomer"
      ).value;
      console.log("Mascota a buscar (Groomer):", mascotaABuscarGroomer);
      buscarCitaPorMascota(mascotaABuscarGroomer);
    }
  }
}

document.addEventListener("DOMContentLoaded", function () {
  const tipoDeUsuario = document.getElementById("tipoUsuario");

  // Agregar evento de cambio al tipo de usuario
  tipoDeUsuario.addEventListener("change", function () {
    // Limpiar el contenedor cuando cambia el tipo de usuario
    limpiarContenedor();

    if (tipoDeUsuario.value === "dueño") {
      mostrarOpcionesDueño();
    } else if (tipoDeUsuario.value === "groomer") {
      mostrarOpcionesGroomer();
    }
  });

  const enviarBtn = document.getElementById("enviarBtn");
  enviarBtn.addEventListener("click", function () {
    procesarFormulario();
  });

  function mostrarOpcionesDueño() {
    opcionesDueño.style.display = "block";
    opcionesGroomer.style.display = "none";
    const accionDueño = document.getElementById("accionDueño").value;

    if (accionDueño === "agregarUsuario") {
      mostrarCampoMascota();
    }
  }

  function mostrarCampoMascota() {
    const nuevoDueñoInput = document.getElementById("nuevoDueño");
    const nuevaMascotaInput = document.getElementById("nuevaMascota");
  }

  function mostrarOpcionesGroomer() {
    opcionesDueño.style.display = "none";
    opcionesGroomer.style.display = "block";
    mostrarCampoMascotaGroomer();
  }

  function mostrarCampoMascotaGroomer() {
    document.getElementById("mascotaABuscarGroomer").style.display = "block";
  }
});
