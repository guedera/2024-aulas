"use client";
import React, { useState, useEffect } from 'react';
import Dropdown from '../_components/dropdown';
import CampoTexto from '../_components/campo-texto';
import BotaoSalvar from '../_components/botao-salvar';
import Titulo from '../_components/titulo';
import Divisoria from '../_components/divisoria';
import styles from './page.module.css';

import { GET, POST } from '../_lib/actions';

export default function Home() {
    const [aparelho, setAparelho] = useState('');
    const [nome, setNome] = useState('');
    const [repeticoes, setRepeticoes] = useState('');
    const [descanso, setDescanso] = useState('');
    const [machineList, setMachineList] = useState([]);
    const [erro, setErro] = useState();

    useEffect(() => {
        async function fetchMachines() {
            try {
                const response = await GET('/maquinas');
                const machines = response.map((machine) => ({
                    value: machine.nome,
                    label: machine.nome,
                }));
                setMachineList(machines);
            } catch (error) {
                setErro("Erro ao conectar ao servidor. Erro: " + error.message);
            }
        }

        fetchMachines();
    }, []);

    const handleSave = async () => {
        const exercicio = {
            nomeDaMaquina: aparelho,
            nome: nome,
            repeticoes: parseInt(repeticoes, 10),
            descanso: parseInt(descanso, 10),
        };

        try {
            await POST('/exercicios', exercicio);
            console.log('Exercício salvo com sucesso:', exercicio);
            setAparelho('');
            setNome('');
            setRepeticoes('');
            setDescanso('');
            setErro('');
        } catch (error) {
            setErro("Erro de salvamento: " + error.message);
        }
    };

    const isFormComplete = aparelho && nome && repeticoes && descanso;

    return (
        <>
            {erro && (
                <div id={styles.mensagemErro}> {erro} </div>
            )}

            <main className={styles.container}>
                <Titulo>Cadastro de exercício</Titulo>
                <Dropdown name="Aparelho" value={aparelho} onChange={setAparelho} list={machineList}/>
                <CampoTexto name="Nome" value={nome} onChange={setNome} />
                <div className={styles.row}>
                    <div>
                        <CampoTexto name="Repetições" value={repeticoes} onChange={setRepeticoes} />
                    </div>
                    <div>
                        <CampoTexto name="Descanso (segundos)" value={descanso} onChange={setDescanso} />
                    </div>
                </div>
                <Divisoria />
                <BotaoSalvar onClick={handleSave} isActive={isFormComplete} />
            </main>
        </>
    );
}
