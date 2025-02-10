"use client";

import { useState, useEffect } from 'react';
import { GET, POST } from '../_lib/actions';
import styles from './page.module.css';
import BotaoAdicionar from '../_components/botao-adicionar';
import SeletorData from '../_components/seletor-data';
import Divisoria from '../_components/divisoria';
import BotaoSalvar from '../_components/botao-salvar';
import SeletorHora from '../_components/seletor-hora';
import DropDown from '../_components/dropdown';
import Titulo from '../_components/titulo';

export default function Home() {
    const [date, setDate] = useState('2024-01-01');
    const [hora, setHora] = useState('05:11');
    const [erro, setErro] = useState('');
    const [telaErro, setTelaErro] = useState(false);
    const [selecionados, setSelecionados] = useState([]); // IDs das séries selecionadas
    const [dropdownList, setDropdownList] = useState([]); // Lista de opções para os dropdowns
    const [isActive, setIsActive] = useState(false);

    useEffect(() => {
        async function fetchData() {
            try {
                const series = await GET('/series');
                const DEx = {};
                const exercicios = await GET('/exercicios');
                exercicios.forEach((exercicio) => {
                    DEx[exercicio.id] = exercicio.nome;
                });

                const lista = series.map((serie) => ({
                    value: serie.idDoExercicio,
                    label: `${DEx[serie.idDoExercicio]} ${serie.numeroDeVezes}x${serie.peso}`,
                }));

                setDropdownList(lista);
            } catch (error) {
                setTelaErro(true);
            }
        }

        fetchData();
    }, []);

    function mudaData(event) {
        setDate(event.target.value);
    }

    function mudaHora(event) {
        setHora(event.target.value);
    }

    function mudaExercicio(value, index) {
        const novoSelecionados = [...selecionados];
        novoSelecionados[index] = value;
        setSelecionados(novoSelecionados);
    }

    function clica() {
        setSelecionados((prev) => [...prev, '']); // Adiciona um novo dropdown vazio
    }

    useEffect(() => {
        const camposPreenchidos = date && hora && selecionados.length > 0;
        setIsActive(camposPreenchidos);
    }, [date, hora, selecionados]);

    const salvar = async () => {
        const payload = { dia: date, hora: hora, series: selecionados };
        const camposPreenchidos = payload.dia && payload.hora && payload.series.length > 0;

        setIsActive(camposPreenchidos);

        if (!camposPreenchidos) {
            alert('Cadastro não finalizado: preencha todos os campos.');
            return;
        }

        setErro('');
        try {
            console.log('Enviando payload:', payload);
            await POST('/treinos', payload);
            alert('Treino salvo com sucesso!');
            setDate('2024-01-01');
            setHora('05:11');
            setSelecionados([]);
        } catch (error) {
            setErro('Erro ao salvar treino: ' + error.message);
        }
    };

    if (telaErro) {
        return (
            <main className={styles.background}>
                <Titulo>Erro ao carregar os dados</Titulo>
                <p>{erro}</p>
            </main>
        );
    }

    return (
        <>
            <main className={styles.backround}>
                <div className={styles.container}>
                    <Titulo>Cadastro de treino</Titulo>
                </div>
                <div className={styles.DateTime}>
                    <div className={styles.container}>
                        <SeletorData name="Dia" value={date} onChange={mudaData} />
                    </div>
                    <div className={styles.container}>
                        <SeletorHora name="Hora" value={hora} onChange={mudaHora} />
                    </div>
                </div>
                <div className={styles.container}>
                    <p>Séries</p>
                    {selecionados.map((value, index) => (
                        <div className={styles.spaceDD} key={index}>
                            <DropDown
                                name=""
                                list={dropdownList}
                                value={value}
                                onChange={(value) => mudaExercicio(value, index)}
                            />
                        </div>
                    ))}
                    <BotaoAdicionar onClick={clica} />
                </div>
                <div className={styles.space}>
                    <Divisoria />
                </div>
                <div className={styles.space}>
                    <BotaoSalvar onClick={salvar} isActive={isActive} />
                </div>
            </main>
            {erro && <p>Erro: {erro}</p>}
        </>
    );
}
