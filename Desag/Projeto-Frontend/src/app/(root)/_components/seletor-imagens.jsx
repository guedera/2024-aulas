"use client"
import Image from "next/image"
import { IoArrowBackCircleOutline , IoArrowForwardCircleOutline } from "react-icons/io5";
import { IconContext } from "react-icons";
import styles from './seletor-imagens.module.css';
import { act, useState, useEffect} from "react";


export default function SeletorImagens(props) {
    const [actual, setActual] = useState(props.list.indexOf(props.value))
    const totalImages = props.list.length
    function incrementa(){
        setActual((actual+1) % totalImages)
    }
    
    function decrementa(){
        setActual(actual > 0 ? (actual-1) : totalImages-1)
        
    }
    
    const {onChange} = props
    useEffect(()=> onChange(props.list[actual]),[actual])
    
    return (
        <div className={styles.seletorContainer}>
            <IconContext.Provider value={{className:styles.seta}}>
                <button className={styles.button} onClick={decrementa}>
                    <IoArrowBackCircleOutline />
                </button>
                    <Image
                        src={props.list[actual]}
                        alt="imagem de máquina"
                        id={styles.maquinaImagem}
                        width={300}
                        height={300}
                    />
                <button className={styles.button} onClick={incrementa}>
                    <IoArrowForwardCircleOutline/>
                </button>
            </IconContext.Provider>
        </div>
    );
}