"use client"; 
import styles from './card-notificacao.module.css';

export default function CardNotificacao(props) {
    return <h1 className={styles.card}>{props.children}</h1>;
  }
  