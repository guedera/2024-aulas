"use client";

import styles from "./page.module.css";
import { GET, POST } from "../_lib/actions"; // Utilitários para requisições ao backend
import { useState, useEffect } from "react";
import Divisoria from "../_components/divisoria";
import BotaoSalvar from "../_components/botao-salvar";
import SeletorImagens from "../_components/seletor-imagens";
import Titulo from "../_components/titulo";
import CampoTexto from "../_components/campo-texto";

export default function Home() {
    const [nome, setNome] = useState(""); // Nome da máquina
    const [pesoMinimo, setPesoMinimo] = useState(""); // Peso mínimo
    const [pesoMaximo, setPesoMaximo] = useState(""); // Peso máximo
    const [imagemSelecionada, setImagemSelecionada] = useState(""); // URL da Foto
    const [maquinas, setMaquinas] = useState([]); // Lista de máquinas carregadas
    const [isActive, setIsActive] = useState(false); // Estado para controlar se o botão está ativo
    const [erro, setErro] = useState(""); // Estado para controlar erros

    // Fetch inicial para carregar as máquinas cadastradas ao montar o componente
    useEffect(() => {
        const fetchData = async () => {
            try {
                const response = await GET("/maquinas"); // Requisição ao backend
                setMaquinas(Array.isArray(response) ? response : []); // Garante que o estado seja uma lista
                console.log("Máquinas carregadas:", response);
            } catch (error) {
                console.error("Erro ao carregar máquinas:", error.message);
                setErro("Erro ao carregar máquinas: " + error.message);
            }
        };

        fetchData();
    }, []);

    // Função para verificar se todos os campos estão preenchidos
    useEffect(() => {
        if (nome && pesoMinimo && pesoMaximo && imagemSelecionada) {
            setIsActive(true); // Ativa o botão quando todos os campos estão preenchidos
        } else {
            setIsActive(false); // Desativa o botão se algum campo estiver vazio
        }
    }, [nome, pesoMinimo, pesoMaximo, imagemSelecionada]);

    const handleSave = async () => {
        if (!nome || !imagemSelecionada || !pesoMinimo || !pesoMaximo) {
            alert("Por favor, preencha todos os campos obrigatórios!");
            return;
        }

        try {
            const body = {
                nome,
                urlDaFoto: imagemSelecionada,
                pesoMinimo: parseFloat(pesoMinimo),
                pesoMaximo: parseFloat(pesoMaximo),
            };

            // Log para verificar os dados antes do envio
            console.log("Dados enviados:", body);

            // Envia os dados para o backend
            const response = await POST("/maquinas", body);

            console.log("Máquina salva com sucesso:", response);
            alert("Máquina cadastrada com sucesso!");

            // Atualiza a lista de máquinas cadastradas após o POST
            setMaquinas((prev) => [...prev, response]);

            // Limpa os campos após o cadastro
            setNome("");
            setPesoMinimo("");
            setPesoMaximo("");
            setImagemSelecionada("");
        } catch (error) {
            console.error("Erro ao salvar a máquina:", error.message);
            setErro("Erro ao salvar a máquina: " + error.message);
        }
    };

    return (
        <main className={styles.container}>
            <Titulo>Cadastro de Máquina</Titulo>

            {erro && <div className={styles.mensagemErro}>{erro}</div>} {/* Exibe o erro se houver */}

            {/* Formulário de cadastro */}
            <CampoTexto
                name="Nome"
                placeholder="Digite o nome da máquina"
                value={nome}
                onChange={setNome}
            />

            <div className={styles.seletorContainer}>
                <SeletorImagens
                    list={maquinas
                        .filter((maquina) => maquina && maquina.urlDaFoto) // Filtra objetos válidos
                        .map((maquina) => maquina.urlDaFoto)} // Usa as imagens carregadas do backend
                    value={imagemSelecionada}
                    onChange={(newImage) => setImagemSelecionada(newImage)}
                />
            </div>

            <div className={styles.row}>
                <CampoTexto
                    name="Peso mínimo (kg)"
                    placeholder="Digite o peso mínimo"
                    value={pesoMinimo}
                    onChange={setPesoMinimo}
                />
                <CampoTexto
                    name="Peso máximo (kg)"
                    placeholder="Digite o peso máximo"
                    value={pesoMaximo}
                    onChange={setPesoMaximo}
                />
            </div>

            <Divisoria />
            <div className={styles.space}>
                <BotaoSalvar onClick={handleSave} isActive={isActive} />
            </div>
        </main>
    );
}
